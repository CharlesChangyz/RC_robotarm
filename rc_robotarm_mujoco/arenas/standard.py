from dm_control import mjcf
from pathlib import Path
import xml.etree.ElementTree as ET
import tempfile


class StandardArena(object):
    def __init__(self, xml_path: str = None) -> None:
        """
        初始化 StandardArena：默认从项目中的 robocon2026.xml 加载 MJCF 场景。

        Args:
            xml_path: 可选的 MJCF/XML 文件路径；默认使用
                rc_robotarm_mujoco/assets/map/robocon2026.xml（相对于本文件）。
        """
        if xml_path is None:
            xml_path = Path(__file__).parent.parent / "assets" / "map" / "robocon2026.xml"

        def _expand_includes(path: Path):
            tree = ET.parse(path)
            root = tree.getroot()

            def _process(elem, base_dir: Path):
                # 将相对 'file' 属性替换为基于 base_dir 的绝对路径
                if 'file' in elem.attrib:
                    val = elem.attrib['file']
                    p = Path(val)
                    if not p.is_absolute():
                        candidate = (base_dir / val)
                        if candidate.exists():
                            abs_path = candidate.resolve()
                        else:
                            candidate2 = (base_dir / 'meshes' / val)
                            if candidate2.exists():
                                abs_path = candidate2.resolve()
                            else:
                                abs_path = candidate.resolve()
                        elem.set('file', str(abs_path))

                # 递归处理子元素并展开 include 标签
                for child in list(elem):
                    if child.tag == 'include' and 'file' in child.attrib:
                        inc_path = (base_dir / child.attrib['file']).resolve()
                        inc_root = ET.parse(inc_path).getroot()
                        _process(inc_root, inc_path.parent)
                        idx = list(elem).index(child)
                        for c in list(inc_root):
                            elem.insert(idx, c)
                            idx += 1
                        elem.remove(child)
                    else:
                        _process(child, base_dir)

            _process(root, path.parent)
            tf = tempfile.NamedTemporaryFile(delete=False, suffix='.xml')
            tree.write(tf.name, encoding='utf-8', xml_declaration=True)
            tf.close()
            return tf.name

        expanded = _expand_includes(Path(xml_path))
        self._mjcf_model = mjcf.from_path(expanded)

        # 保留/覆盖必要的仿真选项
        try:
            self._mjcf_model.option.timestep = 0.002
            self._mjcf_model.option.flag.warmstart = "enable"
        except Exception:
            pass

    def attach(self, child, pos: list = [0, 0, 0], quat: list = [1, 0, 0, 0]) -> mjcf.Element:
        """
        Attaches a child element to the MJCF model at a specified position and orientation.

        Args:
            child: The child element to attach.
            pos: The position of the child element.
            quat: The orientation of the child element.

        Returns:
            The frame of the attached child element.
        """
        frame = self._mjcf_model.attach(child)
        frame.pos = pos
        frame.quat = quat
        return frame
    
    def attach_free(self, child,  pos: list = [0, 0, 0], quat: list = [1, 0, 0, 0]) -> mjcf.Element:
        """
        Attaches a child element to the MJCF model with a free joint.

        Args:
            child: The child element to attach.

        Returns:
            The frame of the attached child element.
        """
        frame = self.attach(child)
        frame.add('freejoint')
        frame.pos = pos
        frame.quat = quat
        return frame
    
    @property
    def mjcf_model(self) -> mjcf.RootElement:
        """
        Returns the MJCF model for the StandardArena object.

        Returns:
            The MJCF model.
        """
        return self._mjcf_model