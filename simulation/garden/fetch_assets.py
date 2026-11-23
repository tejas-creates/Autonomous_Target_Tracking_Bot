#!/usr/bin/env python3
"""Download the CC0 assets used by the garden world into garden/assets/external/.

Only needed once (the files are kept in the repository, so the world works offline).
Every asset is CC0 (public domain); sources are listed in assets/external/CREDITS.md.

    python3 simulation/garden/fetch_assets.py          # skips files that already exist
"""

import hashlib
import io
import json
import shutil
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

EXTERNAL = Path(__file__).resolve().parent / 'assets' / 'external'
AGENT = {'User-Agent': 'gazebo-garden-world-asset-fetch'}

# Poly Haven models (glTF, 1k textures).
POLYHAVEN_MODELS = [
    'planter_pot_clay', 'potted_plant_01', 'potted_plant_02', 'potted_plant_04',
    'tree_stump_01', 'tree_stump_02', 'outdoor_table_chair_set_01',
    'garden_gnome', 'watering_can_metal_01', 'wooden_lantern_01', 'planter_box_01',
    'planter_box_03', 'Barrel_02',
]
# Poly Haven textures: (asset, resolution).
POLYHAVEN_TEXTURES = [
    ('rock_pitted_mossy', '1k'), ('grey_stone_path', '1k'), ('wood_chips', '1k'), ('farm_soil', '1k'),
    ('weathered_planks', '1k'), ('rough_concrete', '1k'), ('brown_planks_04', '1k'), ('bark_brown_02', '1k'),
]
# ambientCG materials (zip with Color / NormalGL / Roughness).
AMBIENTCG = [('Grass004', '2K'), ('Gravel022', '1K')]
# Gazebo Fuel models (owner, name).
FUEL = [('OpenRobotics', 'Oak tree'), ('OpenRobotics', 'Pine Tree')]


def get(url):
    with urllib.request.urlopen(urllib.request.Request(url, headers=AGENT), timeout=120) as r:
        return r.read()


def save(path, data, md5=None):
    if md5 and hashlib.md5(data).hexdigest() != md5:
        raise RuntimeError(f'md5 mismatch for {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def polyhaven_model(asset):
    folder = EXTERNAL / 'polyhaven' / asset
    if (folder / f'{asset}.gltf').exists():
        return
    files = json.loads(get(f'https://api.polyhaven.com/files/{asset}'))
    gltf = files['gltf']['1k']['gltf']
    save(folder / f'{asset}.gltf', get(gltf['url']), gltf.get('md5'))
    for rel, info in gltf['include'].items():
        save(folder / rel, get(info['url']), info.get('md5'))
    print('  polyhaven model', asset)


def polyhaven_texture(asset, res):
    folder = EXTERNAL / 'polyhaven' / asset
    # File names carry the asset name: Gazebo caches textures by file name.
    if (folder / f'{asset}_albedo.jpg').exists():
        return
    files = json.loads(get(f'https://api.polyhaven.com/files/{asset}'))
    for key, name in (('Diffuse', 'albedo'), ('nor_gl', 'normal'), ('Rough', 'roughness')):
        info = files[key][res]['jpg']
        save(folder / f'{asset}_{name}.jpg', get(info['url']), info.get('md5'))
    print('  polyhaven texture', asset)


def ambientcg(asset, res):
    folder = EXTERNAL / 'ambientcg' / asset
    if (folder / f'{asset}_albedo.jpg').exists():
        return
    data = get(f'https://ambientcg.com/get?file={asset}_{res}-JPG.zip')
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for member in z.namelist():
            for suffix, name in (('_Color.jpg', 'albedo'), ('_NormalGL.jpg', 'normal'),
                                 ('_Roughness.jpg', 'roughness')):
                if member.endswith(suffix):
                    save(folder / f'{asset}_{name}.jpg', z.read(member))
    print('  ambientcg', asset)


def fuel(owner, name):
    folder = EXTERNAL / 'fuel' / name.replace(' ', '_')
    if (folder / 'model.sdf').exists():
        return
    base = f'https://fuel.gazebosim.org/1.0/{owner}/models/{urllib.parse.quote(name)}'
    data = get(f'{base}/tip/{urllib.parse.quote(name)}.zip')
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        z.extractall(folder)
    print('  fuel', owner, name)


CREDITS = """# External assets (all CC0 1.0 - public domain, no attribution required)

Downloaded by `simulation/garden/fetch_assets.py`. Credited anyway, with thanks.

| Folder | Source | Author / licence |
|---|---|---|
{rows}
"""


def main():
    rows = []
    for a in POLYHAVEN_MODELS:
        polyhaven_model(a)
        rows.append(f'| polyhaven/{a} | https://polyhaven.com/a/{a} | Poly Haven, CC0 |')
    for a, res in POLYHAVEN_TEXTURES:
        polyhaven_texture(a, res)
        rows.append(f'| polyhaven/{a} | https://polyhaven.com/a/{a} | Poly Haven, CC0 |')
    for a, res in AMBIENTCG:
        ambientcg(a, res)
        rows.append(f'| ambientcg/{a} | https://ambientcg.com/view?id={a} | ambientCG (Lennart Demes), CC0 |')
    for owner, name in FUEL:
        fuel(owner, name)
        rows.append(f'| fuel/{name.replace(" ", "_")} | https://app.gazebosim.org/{owner}/fuel/models/'
                    f'{urllib.parse.quote(name)} | {owner}, CC0 |')
    (EXTERNAL / 'CREDITS.md').write_text(CREDITS.format(rows='\n'.join(rows)))
    size = sum(p.stat().st_size for p in EXTERNAL.rglob('*') if p.is_file())
    print(f'external assets: {size / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
