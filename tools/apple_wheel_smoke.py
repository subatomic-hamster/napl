"""Audit and exercise an installed experimental Apple wheel from an unrelated cwd."""
import argparse
import hashlib
import json
import os
import platform
import re
import struct
import subprocess
import sys
import uuid
import zipfile
import zlib
from pathlib import Path


def command(*args):
    return subprocess.check_output([str(a) for a in args], text=True)


def macho_audit(package):
    records = []
    for path in sorted(package.rglob('*')):
        if path.is_symlink() or not path.is_file():
            continue
        if path.suffix not in ('.so', '.dylib') and path.parent.name != 'bin':
            continue
        arch = command('lipo', '-archs', path).strip()
        assert arch == 'arm64', (path, arch)
        load = command('otool', '-l', path)
        versions, rpaths, identity = [], [], None
        for block in re.split(r'Load command \d+\n', load):
            if 'cmd LC_BUILD_VERSION\n' in block:
                versions += re.findall(r'^\s+minos (\S+)', block, re.M)
            elif 'cmd LC_VERSION_MIN_MACOSX\n' in block:
                versions += re.findall(r'^\s+version (\S+)', block, re.M)
            if 'cmd LC_RPATH\n' in block:
                rpaths += re.findall(r'^\s+path (.*?) \(offset', block, re.M)
            if 'cmd LC_ID_DYLIB\n' in block:
                identity = re.search(r'^\s+name (.*?) \(offset', block, re.M).group(1)
        assert versions and all(tuple(map(int, v.split('.'))) <= (14, 0, 0) for v in versions), (path, versions)
        executable = path.parent if path.parent.name == 'bin' else Path(sys.executable).parent
        def expand(value):
            return Path(value.replace('@loader_path', str(path.parent)).replace('@executable_path', str(executable))).resolve()
        deps = command('otool', '-L', path)
        resolved = {}
        for line in deps.splitlines()[1:]:
            dep = line.strip().split(' (')[0]
            if dep == identity or dep.startswith(('/usr/lib/', '/System/Library/')):
                continue
            if dep.startswith('@rpath/'):
                candidates = [expand(r) / dep[len('@rpath/'):] for r in rpaths]
            elif dep.startswith(('@loader_path/', '@executable_path/')):
                candidates = [expand(dep)]
            else:
                raise AssertionError(f'non-relocatable load path: {path}: {dep}')
            found = next((p.resolve() for p in candidates if p.is_file() and p.resolve().is_relative_to(package)), None)
            assert found is not None, (path, dep, candidates)
            resolved[dep] = str(found.relative_to(package))
        subprocess.run(['codesign', '--verify', '--strict', str(path)], check=True)
        records.append(dict(path=str(path.relative_to(package)), arch=arch, minos=versions,
                            rpaths=rpaths, resolved=resolved, loads=deps))
    assert records
    return records


def write_png(path, image):
    height, width, channels = image.shape
    assert channels == 4
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data))
    pixels = b''.join(b'\0' + row.tobytes() for row in image)
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 6, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(pixels)) + chunk(b'IEND', b''))


def transport_smoke(package, model, output, client, napl):
    name = '/napl-wheel-' + uuid.uuid4().hex[:10]
    log = output / 'shm.naplog'
    server = subprocess.Popen([str(package/'bin/napl-shm-serve'), str(model), '--name', name,
                               '--timeout', '15', '--log', str(log)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        ready = server.stdout.readline()
        assert ready.startswith('ready '), (ready, server.stderr.read())
        response = subprocess.run([str(client), name, '29', '100'], capture_output=True, text=True, check=True, timeout=30)
        stdout, stderr = server.communicate(timeout=20)
        assert server.returncode == 0, (stdout, stderr)
    finally:
        if server.poll() is None:
            server.kill()
            server.wait()
    expected = napl.Sim.load(model)
    mask = (1 << 64) - 1
    for k in range(100):
        z = (29 * 0x9e3779b97f4a7c15 + k * 0xbf58476d1ce4e5b9 + 1) & mask
        z = ((z ^ (z >> 30)) * 0xbf58476d1ce4e5b9) & mask
        z = ((z ^ (z >> 27)) * 0x94d049bb133111eb) & mask
        z ^= z >> 31
        expected.set('/actuators/torque', 'ctrl', (z >> 11) / 9007199254740992.0 * 10.0 - 5.0)
        expected.step(1)
    tick, h = __import__('napl.program', fromlist=['replay_hash']).replay_hash(log, output)
    assert tick == expected.tick == 100 and h == f'{expected.state_hash():016x}'
    assert f'hash={h}' in response.stdout and f'state_hash={h}' in stdout
    return dict(client=response.stdout.strip(), server=stdout.strip(), replay_hash=h)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('wheel', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--shm-client', required=True, type=Path)
    parser.add_argument('--metadata', required=True, type=Path, help='source/build/hardware record prepared before isolating the checkout')
    parser.add_argument('--visible', action='store_true', help='also open and exercise a real Cocoa window')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    import napl
    from napl import program, runtime
    from napl._resources import data_dir, replay_exe, repo_root
    assert platform.machine() == 'arm64' and repo_root() is None
    package = Path(napl.__file__).resolve().parent
    assert package.is_relative_to(Path(sys.prefix).resolve())
    required = ('napl/bin/napl-replay', 'napl/bin/napl-shm-serve', 'napl/bin/napl-px4-bridge', 'napl/bin/napl-view',
                'napl/data/examples/hopper/hop.py', 'napl/data/px4/params/napl_sitl.params',
                'napl/data/notices/ccd/BSD-LICENSE', 'napl/data/notices/fonts/Cousine-OFL.txt')
    with zipfile.ZipFile(args.wheel) as wheel:
        names = set(wheel.namelist())
        for name in required:
            assert name in names, name
        inventory = json.loads(wheel.read('napl/data/asset-inventory.json'))
        for rel, record in inventory['assets'].items():
            data = wheel.read('napl/data/' + rel)
            assert len(data) == record['bytes'] and hashlib.sha256(data).hexdigest() == record['sha256'], rel
    audits = macho_audit(package)
    assert replay_exe().parent == package/'bin'
    result, run_dir = program.run(data_dir()/'examples/hopper/divert25.scenario.toml', args.output/'runs')
    assert result['pass'] and result['replay']['match'], result
    # No checkout fixture: the model is created wholly in the external qualification directory.
    model = args.output/'pendulum.xml'
    model.write_text('<mujoco><option timestep="0.002"/><worldbody><body name="pole" pos="0 0 1">'
                     '<joint name="hinge" type="hinge" axis="0 1 0" damping="0.05"/>'
                     '<geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1"/>'
                     '</body></worldbody><actuator><motor name="torque" joint="hinge" ctrlrange="-5 5"/></actuator></mujoco>')
    sim = napl.Sim.load(model)
    sim.step(37)
    saved = sim.save()
    sim.step(263)
    expected = sim.state_hash()
    sim.restore(saved)
    clone = sim.fork()
    sim.step(263)
    clone.step(263)
    assert sim.tick == clone.tick == 300 and sim.state_hash() == clone.state_hash() == expected
    shm = transport_smoke(package, model, args.output, args.shm_client, napl)
    os.environ['NAPL_OFFSCREEN'] = '1'
    shown = napl.Sim.load(model)
    shown.show(frames=12, script='3 step 5; 10 setattr /world/pole/hinge qpos 0.4', width=480, height=320)
    seq, image = napl._napl.viewer_frame()
    assert seq >= 12 and image.shape == (320,480,4) and (image[...,3] == 255).all()
    assert len(set(map(bytes, image.reshape(-1,4)[::97]))) > 8
    reference = napl.Sim.load(model)
    reference.step(5)
    reference.set('/world/pole/hinge','qpos',[0.4])
    assert shown.state_hash() == reference.state_hash()
    write_png(args.output/'offscreen.png', image)
    os.environ.pop('NAPL_OFFSCREEN')
    if args.visible:
        shown = napl.Sim.load(model)
        shown.show(frames=12, script='3 step 5; 10 setattr /world/pole/hinge qpos 0.4', width=480, height=320)
        assert shown.state_hash() == reference.state_hash()
    notices = {str(p.relative_to(package)):hashlib.sha256(p.read_bytes()).hexdigest()
               for p in (data_dir()/'notices').rglob('*') if p.is_file()}
    manifest = dict(wheel=args.wheel.name, sha256=hashlib.sha256(args.wheel.read_bytes()).hexdigest(),
                    python=sys.version, machine=platform.machine(), os=platform.platform(), policy=runtime.status(),
                    macho=audits, asset_count=len(inventory['assets']), notices=notices, hopper=result,
                    snapshot_hash=f'{expected:016x}', shm=shm, offscreen=True, cocoa=args.visible,
                    build=json.loads(args.metadata.read_text()))
    (args.output/'qualification.json').write_text(json.dumps(manifest, indent=2, default=str)+'\n')
    (args.output/'SHA256SUMS').write_text(f"{manifest['sha256']}  {args.wheel.name}\n")
    print(json.dumps(manifest, indent=2, default=str), flush=True)


if __name__ == '__main__':
    main()
