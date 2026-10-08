"""Barrowman subsonic normal-force slope (CNa) and centre of pressure (CP) of a model rocket, for the `airframe` device's cn_alpha / x_cp.

Source: S. Niskanen, "OpenRocket technical documentation" (v13.05), chapter 3 (Barrowman's method with the sin(a)/a correction dropped for
small angles), reference area Aref = pi d^2 / 4 of the body diameter d. All lengths in m measured from the NOSE TIP aft; CNa per radian.

    eq 3.19   (CNa)_B = 2 [A(l) - A(0)] / Aref          body component of length l (nose cone: 2; a straight tube: 0)
    eq 3.28   X_B     = (l A(l) - V) / (A(l) - A(0))     V = volume; cone 2/3 l, tangent ogive ~0.466 l
    eq 3.40   (CNa)_1 = 2 pi (s^2/Aref) / (1 + sqrt(1 + (s^2 / (A_fin cos Gc))^2))      one fin, M = 0
    eq 3.54   (CNa)_N = (N/2) (CNa)_1 f(N)               f = 1 (N <= 4), 0.948, 0.913, 0.854, 0.810 (N = 5..8), 0.75 (N > 8)
    eq 3.56   K       = 1 + r_t / (s + r_t)              fin-body interference
    eq 3.34   X_f     = Xt (Cr + 2 Ct) / (3 (Cr + Ct)) + (Cr^2 + Ct^2 + Cr Ct) / (6 (Cr + Ct))      from the fin's root leading edge
    eq 3.29   X       = sum X_i (CNa)_i / sum (CNa)_i
Not modelled: Mach / supersonic terms, pitch damping, fin cant, body lift (the airframe device's cross-flow drag s_plan * cd_cross is eq 3.26).
"""

import math

_FIN_FACTOR = {5: 0.948, 6: 0.913, 7: 0.854, 8: 0.810}


def nose(shape: str, length: float, d: float, n: int = 2000):
    """-> (CNa, X from the tip). shape = 'cone' | 'ogive' (tangent ogive)."""
    r0 = 0.5 * d
    if shape == "cone":
        return 2.0, 2.0 * length / 3.0
    if shape != "ogive":
        raise ValueError(f"nose shape {shape!r}: 'cone' or 'ogive'")
    rho = (r0 * r0 + length * length) / (2 * r0)
    r = lambda x: math.sqrt(rho * rho - (length - x) ** 2) + r0 - rho
    h = length / n
    vol = sum(math.pi * r((i + 0.5) * h) ** 2 * h for i in range(n))
    a = math.pi * r0 * r0
    return 2.0, (length * a - vol) / a


def transition(length: float, d_fore: float, d_aft: float, d_ref: float):
    """Conical shoulder / boattail between the diameters d_fore and d_aft (d_fore != d_aft) -> (CNa on Aref = pi d_ref^2/4, X from its front)."""
    a0, a1 = math.pi * d_fore**2 / 4, math.pi * d_aft**2 / 4
    vol = math.pi * length / 12 * (d_fore**2 + d_fore * d_aft + d_aft**2)
    return 2 * (a1 - a0) / (math.pi * d_ref**2 / 4), (length * a1 - vol) / (a1 - a0)


def fins(
    n: int,
    root: float,
    tip: float,
    span: float,
    sweep: float,
    d: float,
    r_body: float | None = None,
):
    """Trapezoidal fin set of n fins -> (CNa on Aref = pi d^2/4, X from the root-chord leading edge).
    root, tip: chords Cr, Ct; span s (root to tip); sweep Xt: leading-edge offset of the tip chord behind the root's; d: reference diameter;
    r_body: body radius at the fins (default d / 2)."""
    if (
        n < 3
    ):  # sum sin^2 = N/2 holds for N >= 3 equally spaced fins (eq 3.53); 1 and 2 fins depend on the roll angle
        raise ValueError("n >= 3 fins")
    aref = math.pi * d * d / 4
    a_fin = 0.5 * (root + tip) * span
    mid = math.atan2(sweep + 0.5 * tip - 0.5 * root, span)  # mid-chord sweep angle Gc
    c1 = (
        2
        * math.pi
        * span
        * span
        / aref
        / (1 + math.sqrt(1 + (span * span / (a_fin * math.cos(mid))) ** 2))
    )
    f = 1.0 if n <= 4 else _FIN_FACTOR.get(n, 0.75)
    rt = 0.5 * d if r_body is None else r_body
    k = 1 + rt / (span + rt)
    x = sweep * (root + 2 * tip) / (3 * (root + tip)) + (
        root**2 + tip**2 + root * tip
    ) / (6 * (root + tip))
    return k * 0.5 * n * c1 * f, x


def cp(parts):
    """[(CNa, X from the tip), ...] -> (total CNa, X of the CP from the tip)."""
    tot = sum(c for c, _ in parts)
    return tot, sum(c * x for c, x in parts) / tot
