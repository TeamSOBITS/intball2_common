#!/usr/bin/env python3
"""経路表示用のvisualization_msgs/Markerを組み立てるヘルパー。"""
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def _point(xyz):
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def _color(rgba):
    return ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))


def _base_marker(frame_id, ns, marker_id, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def make_delete_all_marker():
    """以前に表示したマーカーをすべて消すMarker。"""
    marker = Marker()
    marker.action = Marker.DELETEALL
    return marker


def make_line_strip_marker(frame_id, ns, marker_id, points, colors, width=0.02):
    """points(各[x, y, z])を順に結ぶ線。colorsは各点の色[r, g, b, a]。"""
    marker = _base_marker(frame_id, ns, marker_id, Marker.LINE_STRIP)
    marker.scale.x = float(width)
    marker.points = [_point(p) for p in points]
    marker.colors = [_color(c) for c in colors]
    return marker


def make_arrow_marker(frame_id, ns, marker_id, start, end, color,
                      shaft_diameter=0.02, head_diameter=0.05):
    """startからendへ向かう矢印。"""
    marker = _base_marker(frame_id, ns, marker_id, Marker.ARROW)
    marker.scale.x = float(shaft_diameter)
    marker.scale.y = float(head_diameter)
    marker.scale.z = 0.0
    marker.points = [_point(start), _point(end)]
    marker.color = _color(color)
    return marker


def make_text_marker(frame_id, ns, marker_id, position, text, color, height=0.1):
    """positionに常にカメラの方を向く文字を表示する。"""
    marker = _base_marker(frame_id, ns, marker_id, Marker.TEXT_VIEW_FACING)
    marker.pose.position = _point(position)
    marker.scale.z = float(height)
    marker.color = _color(color)
    marker.text = text
    return marker
