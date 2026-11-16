"""Verify default models and browser resources work from this checkout."""
import importlib.util
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
spec = importlib.util.spec_from_file_location('bundled_web_server', ROOT / 'web/server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class BundledAssetsTests(unittest.TestCase):
    def test_both_robot_urdfs_resolve_all_meshes(self):
        description = ROOT / 'assets/robot/description'
        for name in ('ReBot_Arm_RS.urdf', 'ReBot_Arm_RS_visual.urdf'):
            path = description / 'urdf' / name
            for mesh in ET.parse(path).iter('mesh'):
                filename = mesh.attrib['filename']
                if filename.startswith('package://rebotarm_bringup/description/'):
                    target = description / filename.split('/description/', 1)[1]
                else:
                    target = path.parent / filename
                with self.subTest(urdf=name, mesh=filename):
                    self.assertTrue(target.is_file(), str(target))

    def test_web_assets_are_served_from_checkout(self):
        handler = object.__new__(server.Handler)
        for url in ('/assets/description/urdf/ReBot_Arm_RS_visual.urdf',
                    '/assets/camera/meshes/gemini2/base_link.STL',
                    '/assets/mediapipe/vision_bundle.mjs',
                    '/assets/mediapipe/wasm/vision_wasm_internal.wasm'):
            with self.subTest(url=url):
                target = Path(handler.translate_path(url))
                self.assertTrue(target.is_relative_to(ROOT))
                self.assertTrue(target.is_file())

    def test_controller_files_are_in_checkout(self):
        for name in ('rebot/__init__.py', 'rebot/config.py', 'rebot/controller.py',
                     'config/rebotarm_rs.yaml'):
            self.assertTrue((ROOT / 'vendor/rebot_control' / name).is_file())


if __name__ == '__main__':
    unittest.main()
