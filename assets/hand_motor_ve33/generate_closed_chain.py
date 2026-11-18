#!/usr/bin/env python3
"""Generate the two native SuperDex closed-chain hand prefabs.

Requires numpy and PyYAML. Run without arguments to regenerate both bundled
hands, or pass --bot, --urdf, --closures and --output to convert one model.
The output must be alongside the input bot so its relative asset paths remain
valid. Existing visual-repaired bots are never modified.
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
WITNESS_SEPARATION = 0.03  # meters


def zero_link_poses(urdf_path):
    """Return root-from-link transforms for the URDF's zero joint pose."""
    root = ET.parse(urdf_path).getroot()
    links = {link.attrib['name'] for link in root.findall('link')}
    parents = {}
    for joint in root.findall('joint'):
        child = joint.find('child').attrib['link']
        parent = joint.find('parent').attrib['link']
        if child in parents or child not in links or parent not in links:
            raise ValueError(f'Invalid URDF tree at joint {joint.attrib["name"]}')
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
            raise ValueError(f'Invalid origin for joint {joint.attrib["name"]}')
        cr, cp, cy = np.cos(rpy)
        sr, sp, sy = np.sin(rpy)
        transform = np.eye(4)
        transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]]
        transform[:3, 3] = xyz
        parents[child] = (parent, transform, joint.attrib['name'])
    roots = links - parents.keys()
    if len(roots) != 1:
        raise ValueError(f'URDF must have exactly one root, found {sorted(roots)}')
    poses = {next(iter(roots)): np.eye(4)}
    visiting = set()

    def pose(link):
        if link not in poses:
            if link in visiting or link not in parents:
                raise ValueError(f'Invalid URDF tree at link {link}')
            visiting.add(link)
            parent, transform, _ = parents[link]
            poses[link] = pose(parent) @ transform
            visiting.remove(link)
        return poses[link]

    for link in links:
        pose(link)
    return poses, parents


def convert(bot_path, urdf_path, closures_path, output_path):
    bot_path, urdf_path, closures_path, output_path = map(Path, (bot_path, urdf_path, closures_path, output_path))
    if (output_path.resolve().parent != bot_path.resolve().parent
            or not output_path.name.endswith('_closed_chain.superdex_bot')
            or output_path.resolve() in {p.resolve() for p in (bot_path, urdf_path, closures_path)}):
        raise ValueError('Output must be a separate *_closed_chain.superdex_bot alongside the input bot')
    with bot_path.open(encoding='utf-8') as stream:
        bot = json.load(stream)
    with closures_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    poses, parents = zero_link_poses(urdf_path)
    links, joints = bot['links'], bot['joints']
    indices = {link['name']: i for i, link in enumerate(links)}
    if len(indices) != len(links) or set(indices) != set(poses) or len(joints) != len(links):
        raise ValueError('Bot links/joints do not match the URDF')
    if bot.get('cycles'):
        raise ValueError('Input bot already contains cycles')
    if any(abs(value) > 1e-12 for value in bot.get('defaultPose', [])):
        raise ValueError('Input bot defaultPose must be zero')
    for i, link in enumerate(links):
        name = link['name']
        if name in parents:
            parent, _, joint_name = parents[name]
            if link.get('parentLink') != indices[parent] or joints[i]['name'] != joint_name:
                raise ValueError(f'Bot tree does not match URDF at {name}')
        elif link.get('parentLink', -1) != -1:
            raise ValueError(f'Bot root does not match URDF at {name}')
    cycles = []
    for closure in config['closures']:
        a, b = closure['link_a'], closure['link_b']
        if a not in indices or b not in indices or a == b:
            raise ValueError(f'Invalid closure links: {a}, {b}')
        point = np.asarray(closure['point'], dtype=float)
        axis = np.asarray(closure['axis'], dtype=float)
        if point.shape != (3,) or axis.shape != (3,) or not np.isfinite(np.r_[point, axis]).all():
            raise ValueError(f'Invalid closure point or axis: {a}, {b}')
        norm = np.linalg.norm(axis)
        if norm < 1e-12:
            raise ValueError(f'Zero closure axis: {a}, {b}')
        for witness in (point, point + WITNESS_SEPARATION * axis / norm):
            # URDF closure points are in the zero-pose root frame. The runtime
            # inverts jointFromChildLink to obtain its child-local pivot, then
            # derives the parent pivot from the two links' rest transforms.
            child_pose = poses[b]
            child_local = child_pose[:3, :3].T @ (witness - child_pose[:3, 3])
            cycles.append({'parentLink': indices[a], 'childLink': indices[b],
                           'jointFromChildLink': {'translation': (-child_local).tolist()}})
    bot['cycles'] = cycles
    with output_path.open('w', encoding='utf-8') as stream:
        json.dump(bot, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(f'{output_path}: {len(cycles)} cycle joints')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bot', type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--closures', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    paths = (args.bot, args.urdf, args.closures, args.output)
    if any(paths):
        if not all(paths):
            parser.error('--bot, --urdf, --closures and --output must be supplied together')
        convert(*paths)
    else:
        for side, word in (('R', 'right'), ('L', 'left')):
            directory = HERE / f'hand_motor_ve33_{side}'
            convert(directory / f'hand_motor_ve33_{word}_visual_repaired.superdex_bot',
                    directory / 'hand_tree_only.urdf', directory / 'config/loop_closures.yaml',
                    directory / f'hand_motor_ve33_{word}_closed_chain.superdex_bot')


if __name__ == '__main__':
    main()
