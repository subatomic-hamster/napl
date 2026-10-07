"""fly_unloaded.scenario.toml's model hook: the showcase scene without the payload."""

import showcase_scene


def build(outdir):
    return showcase_scene.build(outdir, payload=False)
