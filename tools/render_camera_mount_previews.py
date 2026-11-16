#!/usr/bin/env python3
"""Render consistent previews directly from the two bundled STL meshes."""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MOUNTS = [
    (ROOT / 'models/wrist_camera/gemini2-gemini336-d435i/Gemini2_Gemini336_D435i_Mount.stl',
     ROOT / 'models/wrist_camera/gemini2-gemini336-d435i/Gemini2_Gemini336_D435i_Mount_preview.png',
     'Gemini 2 · Gemini 336 · D435i', '相机共用支架'),
    (ROOT / 'models/wrist_camera/meshes/Gemini305_D405_Mount.stl',
     ROOT / 'models/wrist_camera/gemini305-d405/Gemini305_D405_Mount_preview.png',
     'Gemini 305 · D405', '相机共用支架'),
]
STL_RECORD = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
font_files = font_manager.findSystemFonts()
cjk_font = next((path for path in font_files if Path(path).name == 'NotoSansCJK-Regular.ttc'), None)
if cjk_font is None:
    raise RuntimeError('请安装 Noto Sans CJK 字体后生成预览图')
font_manager.fontManager.addfont(cjk_font)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=cjk_font).get_name()
plt.rcParams['axes.unicode_minus'] = False


def read_stl(path):
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f'无效 STL: {path}')
    count = int.from_bytes(raw[80:84], 'little')
    if len(raw) != 84 + 50 * count or not count:
        raise ValueError(f'仅支持有效的二进制 STL: {path}')
    records = np.frombuffer(raw, dtype=STL_RECORD, count=count, offset=84)
    triangles = records['vertices'].astype(float)
    if not np.isfinite(triangles).all():
        raise ValueError(f'STL 包含无效坐标: {path}')
    return triangles


def draw_view(fig, rect, triangles, elev, azim):
    ax = fig.add_axes(rect, projection='3d')
    ax.set_facecolor('#f8fafb')
    edges_a = triangles[:, 1] - triangles[:, 0]
    edges_b = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edges_a, edges_b)
    length = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(length[:, None], 1e-12)
    light = np.array([-0.45, -0.65, 0.85]); light /= np.linalg.norm(light)
    brightness = .48 + .48 * np.abs(normals @ light)
    base = np.array([.055, .43, .51])
    colors = np.clip(brightness[:, None] * base[None, :] + .08, 0, 1)
    mesh = Poly3DCollection(triangles, facecolors=colors, edgecolors='none', linewidths=0, antialiased=False)
    ax.add_collection3d(mesh)
    vertices = triangles.reshape(-1, 3)
    lo, hi = vertices.min(axis=0), vertices.max(axis=0)
    center = (lo + hi) / 2
    radius = max(hi - lo) * .56
    for setter, middle in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
        setter(middle - radius, middle + radius)
    # Leave room around the mesh so its silhouette is not clipped by the axes.
    ax.set_box_aspect((1, 1, 1), zoom=1.45)
    ax.set_proj_type('ortho')
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    return hi - lo


def render(mesh_path, output_path, title, subtitle):
    triangles = read_stl(mesh_path)
    fig = plt.figure(figsize=(16, 10), dpi=100, facecolor='#eff3f5')
    fig.text(.055, .925, 'CAMERA MOUNT  /  支架预览', fontsize=13, weight='medium', color='#54818a')
    fig.text(.055, .865, title, fontsize=31, weight='bold', color='#142b36')
    fig.text(.055, .822, subtitle + '  ·  基于 STL 三维模型', fontsize=15, color='#586f7d')
    for x, y, w, h in ((.055, .215, .56, .56), (.635, .485, .31, .29), (.635, .215, .31, .25)):
        fig.add_artist(FancyBboxPatch((x, y), w, h, transform=fig.transFigure,
                       boxstyle='round,pad=0.006,rounding_size=0.014',
                       linewidth=0, facecolor='#f8fafb', zorder=0))
    fig.text(.085, .73, '立体视图', fontsize=15, weight='bold', color='#30505c')
    fig.text(.665, .73, '俯视图', fontsize=14, weight='bold', color='#30505c')
    fig.text(.665, .43, '侧视图', fontsize=14, weight='bold', color='#30505c')
    size = draw_view(fig, [.075, .24, .52, .46], triangles, 25, -58)
    draw_view(fig, [.66, .5, .26, .21], triangles, 90, -90)
    draw_view(fig, [.66, .225, .26, .18], triangles, 0, -90)
    fig.text(.055, .145, f'模型包围盒  X {size[0]:.1f} × Y {size[1]:.1f} × Z {size[2]:.1f} mm',
             fontsize=16, weight='medium', color='#1d4650')
    fig.text(.055, .095, '尺寸为 STL 坐标轴方向的外接范围；安装孔位及间隙以模型和实物为准。',
             fontsize=13, color='#687f8b')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f'{output_path}: {size.round(2).tolist()} mm')


if __name__ == '__main__':
    for item in MOUNTS:
        render(*item)
