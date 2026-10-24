"""Generate the Apple wheel's runtime asset inventory and exact CMake install rules.

Ship a conservative closure (all source files beneath runtime roots), rather than an
extension allowlist that silently drops uppercase STL or future controller assets.
"""
import hashlib
import json
import sys
import tomllib
from pathlib import Path

ROOTS = ('parts', 'assemblies', 'meshes', 'examples', 'cad', 'firmware', 'px4', 'docs')
EXCLUDED_DIRS = {'__pycache__', 'build', 'runs', 'out', 'reports', '.venv', 'node_modules'}
EXCLUDED_SUFFIXES = {'.pyc', '.pyo', '.o', '.a', '.so', '.dylib', '.naplog'}
EVIDENCE = {'docs/models.md', 'docs/validation/thrust-stands.md', 'docs/validation/x500-hover.md'}


def generate(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    assets = {}
    for name in ROOTS:
        root = source / name
        if not root.is_dir():
            raise FileNotFoundError(f'missing runtime asset root: {root}')
        for path in sorted(root.rglob('*')):
            rel = path.relative_to(source)
            if name == 'docs' and rel.as_posix() not in EVIDENCE:
                continue
            if any(p.startswith('.') or p in EXCLUDED_DIRS for p in rel.parts):
                continue
            if path.is_dir() or path.suffix.lower() in EXCLUDED_SUFFIXES:
                continue
            if not path.resolve().is_relative_to(source):
                raise ValueError(f'asset symlink escapes source tree: {rel}')
            data = path.read_bytes()
            if data.startswith(b'version https://git-lfs.github.com/spec/v1\n'):
                raise ValueError(f'unresolved Git LFS runtime asset: {rel}')
            assets[rel.as_posix()] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())

    edges = []
    def reference(owner, target):
        rel = target.relative_to(source).as_posix()
        if rel not in assets:
            raise FileNotFoundError(f'{owner}: missing packaged dependency {rel}')
        edges.append(dict(source=owner, target=rel))

    def geometry(owner, value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key.startswith('mesh') and isinstance(item, str) and item.startswith('meshes/'):
                    reference(owner, source / item)
                geometry(owner, item)
        elif isinstance(value, list):
            for item in value:
                geometry(owner, item)

    for rel in assets:
        if rel.startswith(('parts/', 'assemblies/')) and rel.endswith('.toml'):
            doc = tomllib.loads((source / rel).read_text())
            geometry(rel, doc)
            for sub in doc.get('subassemblies', []):
                if 'file' in sub:
                    reference(rel, ((source / rel).parent / sub['file']).resolve())
        if rel.startswith('examples/') and rel.endswith('.scenario.toml'):
            doc = tomllib.loads((source / rel).read_text())
            parent = (source / rel).parent
            if script := doc.get('model', {}).get('script'):
                reference(rel, (parent / script).resolve())
            for arg in doc.get('program', {}).get('cmd', []):
                if arg.endswith('.py'):
                    value = arg.replace('{dir}', str(parent))
                    if '{' not in value:
                        reference(rel, (parent / value).resolve())
    for required in (*sorted(EVIDENCE), 'px4/params/napl_sitl.params'):
        reference('napl runtime', source / required)

    destination.mkdir(parents=True, exist_ok=True)
    manifest = dict(schema=1, strategy='conservative-source-closure', roots=ROOTS,
                    assets=assets, references=edges)
    (destination / 'asset-inventory.json').write_text(json.dumps(manifest, indent=2) + '\n')
    quote = lambda text: '"' + str(text).replace('\\', '/').replace('"', '\\"').replace(';', '\\;') + '"'
    rules = []
    for rel in assets:
        rules.append(f'install(FILES {quote(source / rel)} DESTINATION {quote(Path("napl/data") / Path(rel).parent)} COMPONENT napl_python)')
        rules.append(f'set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS {quote(source / rel)})')
    (destination / 'wheel-assets.cmake').write_text('\n'.join(rules) + '\n')
    print(f'Wheel asset inventory: {len(assets)} files, {sum(v["bytes"] for v in assets.values())} bytes, {len(edges)} validated references')


if __name__ == '__main__':
    generate(*sys.argv[1:])
