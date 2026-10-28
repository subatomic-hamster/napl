"""Residual motion predictors over the IMM2 extrapolation: MLP and GRU with a heteroscedastic Gaussian head.

Output per horizon h: mean = IMM2 mean + rs_h * m_h (3), covariance = L L^T with L = chol(IMM2 predicted cov) whose diagonal is
scaled by exp(d_h) (3) and whose lower triangle gets o_h * mean diag (3). The last layer is zero-initialised, so training
starts exactly at the IMM2 prediction and cannot be worse than it by construction at step 0. Trained by Gaussian NLL.
Inference is plain numpy (forward_np); torch is needed only for train()."""

import os

import numpy as np

from .metrics import tri_to_cov

LOW = ((1, 0), (2, 0), (2, 1))
BASE_FLOOR = (
    1e-4  # m^2 added to the IMM covariance before its Cholesky factor (1 cm std)
)


def _ssqrt(x):
    return np.sign(x) * np.sqrt(np.abs(x))


def chol_base(pc):
    return np.linalg.cholesky(
        tri_to_cov(np.asarray(pc, np.float64)) + BASE_FLOOR * np.eye(3)
    )


def featurize(d, Lb=None):
    """dict of arrays (hm hs hu hn cc pm2 pc2 meta) -> (seq (S, N, 15), static (S, F)) float32, unnormalised."""
    hm, N = np.asarray(d["hm"], np.float64), d["hm"].shape[1]
    cur = hm[:, -1, :3]
    rel = hm.copy()
    rel[..., :3] = (rel[..., :3] - cur[:, None]) / 5
    rel[..., 3:] /= 10
    valid = (np.arange(N)[None] >= N - np.asarray(d["hn"])[:, None]).astype(np.float64)
    seq = np.concatenate([rel, np.log(d["hs"] + 1e-3), d["hu"], valid[..., None]], -1)
    Lb = chol_base(d["pc2"]) if Lb is None else Lb
    disp = (np.asarray(d["pm2"], np.float64) - cur[:, None]) / 10
    ld = np.log(np.stack([Lb[..., i, i] for i in range(3)], -1))
    off = _ssqrt(np.stack([Lb[..., i, j] for i, j in LOW], -1))
    S = len(cur)
    static = np.concatenate(
        [
            _ssqrt(d["cc"]),
            disp.reshape(S, -1),
            ld.reshape(S, -1),
            off.reshape(S, -1),
            d["meta"][:, 4:5] / 0.1,
        ],
        -1,
    )
    return seq.astype(np.float32), static.astype(np.float32)


def decode(out, Lb, pm2, rs):
    """Network output (S, H, 9) -> mean (S, H, 3), cov (S, H, 3, 3), numpy."""
    mean = pm2 + rs[None, :, None] * out[..., :3]
    dg = np.exp(np.clip(out[..., 3:6], -4, 4))
    sc = np.mean([Lb[..., i, i] for i in range(3)], 0)
    L = Lb.copy()
    for k in range(3):
        L[..., k, k] = Lb[..., k, k] * dg[..., k]
    for k, (i, j) in enumerate(LOW):
        L[..., i, j] = Lb[..., i, j] + out[..., 6 + k] * sc
    return mean, L @ np.swapaxes(L, -1, -2)


def _silu(x):
    return x / (1 + np.exp(-x))


def _sig(x):
    return 1 / (1 + np.exp(-x))


def forward_np(W, seq, static):
    """Numpy forward of a net exported by export() -> (S, H*9). W: dict of float32 arrays + 'kind'."""
    seq = (seq - W["s_mu"]) / W["s_sd"]
    st = (static - W["f_mu"]) / W["f_sd"]
    if W["kind"] == "gru":
        h = np.zeros((len(seq), W["w_hh"].shape[1]), np.float32)
        g = h.shape[1]
        for t in range(seq.shape[1]):
            gi, gh = seq[:, t] @ W["w_ih"].T + W["b_ih"], h @ W["w_hh"].T + W["b_hh"]
            r, z = (
                _sig(gi[:, :g] + gh[:, :g]),
                _sig(gi[:, g : 2 * g] + gh[:, g : 2 * g]),
            )
            n = np.tanh(gi[:, 2 * g :] + r * gh[:, 2 * g :])
            h = (1 - z) * n + z * h
        x = np.concatenate([h, st], -1)
    else:
        x = np.concatenate([seq.reshape(len(seq), -1), st], -1)
    x = _silu(x @ W["w1"].T + W["b1"])
    x = _silu(x @ W["w2"].T + W["b2"])
    return x @ W["w3"].T + W["b3"]


def predict_np(W, d, bs=20000):
    """Learned model on a dataset split -> mean (S, H, 3), cov (S, H, 3, 3)."""
    Lb = chol_base(d["pc2"])
    seq, static = featurize(d, Lb)
    H = d["y"].shape[1]
    out = np.concatenate(
        [
            forward_np(W, seq[i : i + bs], static[i : i + bs])
            for i in range(0, len(seq), bs)
        ]
    ).reshape(len(seq), H, 9)
    return decode(out, Lb, np.asarray(d["pm2"], np.float64), W["rs"])


def train(
    tr,
    va,
    kind="mlp",
    seed=0,
    hidden=128,
    gru=48,
    epochs=40,
    bs=4096,
    lr=2e-3,
    device="cuda",
    log=print,
):
    """Fit on split tr, early-select on split va (val NLL). Returns the numpy weight dict W (see forward_np)."""
    import torch
    from torch import nn

    torch.manual_seed(seed)
    dev = torch.device(device if torch.cuda.is_available() else "cpu")
    if dev.type == "cuda":
        torch.cuda.set_per_process_memory_fraction(
            float(os.environ.get("NAPL_GPU_FRAC", "0.1"))
        )  # of 16 GB: default well under the 4 GB GTT rule; 0.2 = 3.2 GB for the ML-3 set (1M samples resident)
    H = tr["y"].shape[1]

    def prep(d):
        Lb = chol_base(d["pc2"])
        seq, st = featurize(d, Lb)
        return seq, st, Lb

    s_tr, f_tr, L_tr = prep(tr)
    s_va, f_va, L_va = prep(va)
    W0 = dict(
        kind=kind,
        s_mu=s_tr.reshape(-1, s_tr.shape[-1]).mean(0),
        s_sd=s_tr.reshape(-1, s_tr.shape[-1]).std(0) + 1e-3,
        f_mu=f_tr.mean(0),
        f_sd=f_tr.std(0) + 1e-3,
        rs=np.float64(
            np.sqrt(((tr["y"] - tr["pm2"]).astype(np.float64) ** 2).mean((0, 2)))
        ),
    )
    nseq, nf = s_tr.shape[1:], f_tr.shape[1]

    class Net(nn.Module):
        def __init__(s):
            super().__init__()
            if kind == "gru":
                s.rnn = nn.GRU(nseq[1], gru, batch_first=True)
                din = gru + nf
            else:
                din = nseq[0] * nseq[1] + nf
            s.head = nn.Sequential(
                nn.Linear(din, hidden),
                nn.SiLU(),
                nn.Linear(hidden, hidden),
                nn.SiLU(),
                nn.Linear(hidden, H * 9),
            )
            nn.init.zeros_(s.head[-1].weight), nn.init.zeros_(s.head[-1].bias)

        def forward(s, q, f):
            x = s.rnn(q)[1][0] if kind == "gru" else q.flatten(1)
            return s.head(torch.cat([x, f], -1)).view(-1, H, 9)

    net = Net().to(dev)

    def T(a, dt=torch.float32):
        return torch.as_tensor(np.ascontiguousarray(a), dtype=dt, device=dev)

    def pack(d, s, f, Lb):
        return dict(
            q=T((s - W0["s_mu"]) / W0["s_sd"]),
            f=T((f - W0["f_mu"]) / W0["f_sd"]),
            Lb=T(Lb),
            pm=T(d["pm2"]),
            y=T(d["y"]),
        )

    P_tr, P_va = pack(tr, s_tr, f_tr, L_tr), pack(va, s_va, f_va, L_va)
    rs = T(W0["rs"])

    def nll(P, idx):
        o = net(P["q"][idx], P["f"][idx])
        Lb = P["Lb"][idx]
        dg = torch.exp(o[..., 3:6].clamp(-4, 4))
        sc = torch.diagonal(Lb, dim1=-2, dim2=-1).mean(-1, keepdim=True)
        L = torch.diag_embed(torch.diagonal(Lb, dim1=-2, dim2=-1) * dg)
        for k, (i, j) in enumerate(LOW):
            L[..., i, j] = Lb[..., i, j] + o[..., 6 + k] * sc[..., 0]
        e = P["y"][idx] - (P["pm"][idx] + rs[None, :, None] * o[..., :3])
        z = torch.linalg.solve_triangular(L, e[..., None], upper=False)[..., 0]
        return (
            0.5 * (z**2).sum(-1)
            + torch.log(torch.diagonal(L, dim1=-2, dim2=-1)).sum(-1)
        ).mean()

    opt = torch.optim.AdamW(net.parameters(), lr=lr, weight_decay=1e-5)
    n = len(P_tr["y"])
    steps = epochs * ((n + bs - 1) // bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(
        opt, max_lr=lr, total_steps=steps, pct_start=0.1
    )
    best, best_sd = (1e30, -1), None
    g = torch.Generator(device="cpu").manual_seed(seed)
    for ep in range(epochs):
        net.train()
        perm = torch.randperm(n, generator=g).to(dev)
        for i in range(0, n, bs):
            loss = nll(P_tr, perm[i : i + bs])
            opt.zero_grad(), loss.backward()
            nn.utils.clip_grad_norm_(net.parameters(), 5.0)
            opt.step(), sched.step()
        net.eval()
        with torch.no_grad():
            nv = len(P_va["y"])
            v = (
                sum(
                    float(nll(P_va, torch.arange(i, min(i + 8192, nv), device=dev)))
                    * (min(i + 8192, nv) - i)
                    for i in range(0, nv, 8192)
                )
                / nv
            )
        if v < best[0]:
            best, best_sd = (
                (v, ep),
                {
                    k: x.detach().cpu().numpy().copy()
                    for k, x in net.state_dict().items()
                },
            )
        if ep % 5 == 0 or ep == epochs - 1:
            log(
                f"{kind} seed {seed} ep {ep} train {float(loss.detach()):.3f} val {v:.3f} (best {best[0]:.3f} @ {best[1]})"
            )
    sd = best_sd
    W = dict(W0)
    for n_, li in (("1", 0), ("2", 2), ("3", 4)):
        W["w" + n_], W["b" + n_] = sd[f"head.{li}.weight"], sd[f"head.{li}.bias"]
    if kind == "gru":
        W.update(
            w_ih=sd["rnn.weight_ih_l0"],
            w_hh=sd["rnn.weight_hh_l0"],
            b_ih=sd["rnn.bias_ih_l0"],
            b_hh=sd["rnn.bias_hh_l0"],
        )
    W["val_nll"], W["best_epoch"] = best[0], best[1]
    del (
        net,
        opt,
        sched,
        P_tr,
        P_va,
    )  # closures hold reference cycles: free the GPU before the next seed
    import gc

    gc.collect()
    if dev.type == "cuda":
        torch.cuda.empty_cache()
    return W


def save(path, W):
    np.savez(path, **{k: np.asarray(v) for k, v in W.items()})


def load(path):
    z = np.load(path)
    W = {k: z[k] for k in z.files}
    W["kind"] = str(W["kind"])
    return W
