"""Viser front end. Meshes are placed with hexapod_kinematics.link_frames, so the view uses the same FK as everything else."""
from __future__ import annotations

import threading
import time

import numpy as np
import trimesh
import viser
import viser.transforms as vtf

from hexapod_kinematics import (DEFAULT_STEP_HEIGHT, DEFAULT_STEP_LENGTH, DEFAULTS, JOINT_ORDER, GaitGenerator, GaitParams, RobotModel,
                                VelocityCommand, link_frames)

from .logic import Frame, body_frame, gait_frame, joint_frame, nearest_point_on_polygon, pattern_q, status_html, table_html

MODES = ('Forward kinematics', 'Body pose IK', 'Leg patterns', 'Gait playback')
COLOURS = {'base': (201, 106, 106), 'coxa': (111, 201, 196), 'femur': (127, 208, 127), 'tibia': (106, 106, 201), 'foot': (40, 40, 40)}
GAIT_DT = 0.02
TICK = 1.0 / 30.0


def _load_mesh(path) -> trimesh.Trimesh:
    return trimesh.load_mesh(path, force='mesh')


class HexapodSim:
    def __init__(self, model: RobotModel, host: str = '127.0.0.1', port: int = 8080, mode: str = MODES[0]):
        self.model = model
        self.server = viser.ViserServer(host=host, port=port, verbose=False)
        self.lock = threading.Lock()
        self.dirty = True
        self.running = True
        self.frame: Frame | None = None
        self.start_q = model.pose_q('standby')
        self._setup_scene()
        self._setup_gui(mode)
        self.generator = GaitGenerator(model, self._gait_params())
        self.playing = False
        self.pending_steps = 0
        self.last_sample = None
        self.accumulator = 0.0

    # ---- scene ----
    def _setup_scene(self) -> None:
        s = self.server.scene
        s.set_up_direction('+z')
        self.server.gui.configure_theme(control_width='large', show_logo=False, show_share_button=False)
        self.server.gui.main_panel.dock_right()
        self.grid = s.add_grid('/ground', width=4.0, height=4.0, cell_size=0.05, section_size=0.25,
                               cell_color=(215, 215, 215), section_color=(165, 165, 165), plane='xy')
        s.add_light_hemisphere('/light', sky_color=(255, 255, 255), ground_color=(120, 120, 120), intensity=1.2)
        self.robot = s.add_frame('/robot', show_axes=False)
        directory = self.model.mesh_dir
        base = _load_mesh(directory / 'base_link.stl')
        s.add_mesh_simple('/robot/base', base.vertices, base.faces, color=COLOURS['base'], flat_shading=True)
        self.link_handles = {}
        cache = {}
        for leg in self.model.legs:
            for name in ('coxa', 'femur', 'tibia', 'foot'):
                file = f'{name}_{leg.side}'
                if file not in cache:
                    cache[file] = _load_mesh(directory / f'{file}.stl')
                mesh = cache[file]
                self.link_handles[(leg.index, name)] = s.add_mesh_simple(
                    f'/robot/leg_{leg.id}_{name}', mesh.vertices, mesh.faces, color=COLOURS[name], flat_shading=True)
        self.com_handle = s.add_icosphere('/com', radius=0.006, color=(230, 120, 0), position=(0.0, 0.0, 0.002))
        self.foot_handles = [s.add_icosphere(f'/foot_marker_{leg.id}', radius=0.005, color=(30, 110, 230)) for leg in self.model.legs]

        @self.server.on_client_connect
        def _on_connect(client):
            client.camera.position = (0.27, -0.33, 0.25)
            client.camera.look_at = (0.0, 0.0, 0.03)
            client.camera.up_direction = (0.0, 0.0, 1.0)

    # ---- GUI ----
    def _slider(self, label, low, high, step, value, hint=None):
        handle = self.server.gui.add_slider(label, min=low, max=high, step=step, initial_value=value, hint=hint)
        handle.on_update(self._mark_dirty)
        return handle

    def _mark_dirty(self, _event=None) -> None:
        self.dirty = True

    def _setup_gui(self, mode: str) -> None:
        gui = self.server.gui
        gui.add_markdown('**Hexapod simulator** (RookiDroid Mochi)')
        self.mode = gui.add_dropdown('Mode', MODES, initial_value=mode)
        self.mode.on_update(self._on_mode)
        self.status = gui.add_html('')
        self.table = gui.add_html('')
        self.panels = {}
        self._setup_move_check()
        self._setup_fk()
        self._setup_body()
        self._setup_pattern()
        self._setup_gait()
        self._apply_mode_visibility()

    def _setup_move_check(self) -> None:
        gui = self.server.gui
        self.move_folder = gui.add_folder('Servo speed check')
        with self.move_folder:
            self.move_time = self._slider('Move time (s)', 0.02, 2.0, 0.01, 0.2, hint='Warns if the move from the start pose to this pose needs more than the servo speed.')
            button = gui.add_button('Use this pose as start')
            button.on_click(self._on_mark_start)

    def _on_mark_start(self, _event=None) -> None:
        if self.frame is not None:
            self.start_q = self.frame.q.copy()
        self.dirty = True

    def _setup_fk(self) -> None:
        gui = self.server.gui
        standby = np.degrees(self.model.poses['standby'])
        self.fk_sliders = {}
        panel = gui.add_folder('Joint angles')
        self.panels[MODES[0]] = panel
        with panel:
            for leg in self.model.legs:
                with gui.add_folder(f'Leg {leg.id}: {leg.name}', expand_by_default=leg.index == 0):
                    for j, name in enumerate(JOINT_ORDER):
                        hint = f'Limit {np.degrees(self.model.lower[j]):.0f}° to {np.degrees(self.model.upper[j]):.0f}°. The slider goes further so you can see the warning.'
                        self.fk_sliders[(leg.index, j)] = self._slider(f'{name} (°)', -90.0, 90.0, 0.5, float(round(standby[j], 1)), hint)

    def _setup_body(self) -> None:
        panel = self.server.gui.add_folder('Body pose')
        self.panels[MODES[1]] = panel
        self.body_sliders = {}
        with panel:
            for key, label, low, high, step in (('x', 'x (mm)', -80.0, 80.0, 1.0), ('y', 'y (mm)', -80.0, 80.0, 1.0), ('z', 'z (mm)', -60.0, 60.0, 1.0),
                                                ('roll', 'roll (°)', -40.0, 40.0, 0.5), ('pitch', 'pitch (°)', -40.0, 40.0, 0.5), ('yaw', 'yaw (°)', -40.0, 40.0, 0.5)):
                self.body_sliders[key] = self._slider(label, low, high, step, 0.0)
            reset = self.server.gui.add_button('Reset body pose')
            reset.on_click(self._on_body_reset)

    def _on_body_reset(self, _event=None) -> None:
        for handle in self.body_sliders.values():
            handle.value = 0.0
        self.dirty = True

    def _setup_pattern(self) -> None:
        gui = self.server.gui
        panel = gui.add_folder('Same angles on all legs')
        self.panels[MODES[2]] = panel
        standby = np.degrees(self.model.poses['standby'])
        self.pattern_sliders = {}
        with panel:
            for j, name in enumerate(JOINT_ORDER):
                self.pattern_sliders[j] = self._slider(f'{name} (°)', -90.0, 90.0, 0.5, float(round(standby[j], 1)))
            for pose in self.model.poses:
                button = gui.add_button(f'Pose: {pose}')
                button.on_click(lambda _e, p=pose: self._on_pattern_pose(p))

    def _on_pattern_pose(self, pose: str) -> None:
        for j, value in enumerate(np.degrees(self.model.poses[pose])):
            self.pattern_sliders[j].value = float(round(value, 1))
        self.dirty = True

    def _setup_gait(self) -> None:
        gui = self.server.gui
        panel = gui.add_folder('Gait')
        self.panels[MODES[3]] = panel
        with panel:
            self.gait_type = gui.add_dropdown('Gait', tuple(sorted(DEFAULTS)), initial_value='tripod')
            self.gait_type.on_update(self._on_gait_type)
            self.step_length = self._slider('Step length (mm)', 5.0, 80.0, 1.0, DEFAULT_STEP_LENGTH * 1000)
            self.step_height = self._slider('Step height (mm)', 3.0, 50.0, 1.0, DEFAULT_STEP_HEIGHT * 1000)
            self.period = self._slider('Cycle period (s)', 0.3, 12.0, 0.1, DEFAULTS['tripod']['cycle_period'])
            self.duty = self._slider('Duty factor', 0.3, 0.95, 0.01, DEFAULTS['tripod']['duty_factor'], hint='Changing this restarts the gait from standing.')
            self.duty.on_update(lambda _e: self._reset_gait())
            self.vx = self._slider('vx (mm/s)', -60.0, 60.0, 1.0, 15.0)
            self.vy = self._slider('vy (mm/s)', -60.0, 60.0, 1.0, 0.0)
            self.yaw_rate = self._slider('yaw rate (rad/s)', -0.6, 0.6, 0.01, 0.0)
            self.speed = self._slider('Playback speed', 0.1, 3.0, 0.1, 1.0)
            self.follow = gui.add_checkbox('Camera follows the robot', True)
            self.play_buttons = gui.add_button_group('Playback', ('Play', 'Pause', 'Step', 'Reset'))
            self.play_buttons.on_click(self._on_playback)

    def _gait_params(self) -> GaitParams:
        return GaitParams(gait=self.gait_type.value, step_length=self.step_length.value / 1000.0, step_height=self.step_height.value / 1000.0,
                          cycle_period=self.period.value, duty_factor=self.duty.value)

    def _command(self) -> VelocityCommand:
        return VelocityCommand(self.vx.value / 1000.0, self.vy.value / 1000.0, self.yaw_rate.value)

    def _on_gait_type(self, _event=None) -> None:
        defaults = DEFAULTS[self.gait_type.value]
        self.period.value = defaults['cycle_period']
        self.duty.value = defaults['duty_factor']
        self._reset_gait()

    def _reset_gait(self) -> None:
        with self.lock:
            self.generator = GaitGenerator(self.model, self._gait_params())
            self.last_sample = None
        self.dirty = True

    def _on_playback(self, event) -> None:
        choice = event.target.value
        if choice == 'Play':
            self.playing = True
        elif choice == 'Pause':
            self.playing = False
        elif choice == 'Step':
            self.playing = False
            self.pending_steps += 1
        elif choice == 'Reset':
            self.playing = False
            self._reset_gait()
        self.dirty = True

    def _on_mode(self, _event=None) -> None:
        self.playing = False
        self._apply_mode_visibility()
        if self.frame is not None:
            self.start_q = self.frame.q.copy()
        if self.mode.value == MODES[3]:
            self._reset_gait()
        self.dirty = True

    def _apply_mode_visibility(self) -> None:
        for name, panel in self.panels.items():
            panel.visible = name == self.mode.value
        self.move_folder.visible = self.mode.value != MODES[3]

    # ---- frames ----
    def compute_frame(self) -> Frame:
        mode = self.mode.value
        if mode == MODES[0]:
            q = np.array([[np.radians(self.fk_sliders[(leg.index, j)].value) for j in range(3)] for leg in self.model.legs])
            return joint_frame(self.model, q, self.start_q, self.move_time.value)
        if mode == MODES[1]:
            b = self.body_sliders
            translation = (b['x'].value / 1000.0, b['y'].value / 1000.0, b['z'].value / 1000.0)
            rpy = tuple(np.radians(b[k].value) for k in ('roll', 'pitch', 'yaw'))
            return body_frame(self.model, translation, rpy, self.start_q, self.move_time.value)
        if mode == MODES[2]:
            q = pattern_q(self.model, *(np.radians(self.pattern_sliders[j].value) for j in range(3)))
            return joint_frame(self.model, q, self.start_q, self.move_time.value)
        return self._gait_step()

    def _gait_step(self) -> Frame:
        with self.lock:
            generator = self.generator
            generator.params = self._gait_params()
            command = self._command()
            steps = self.pending_steps
            self.pending_steps = 0
            if self.playing:
                self.accumulator += TICK * self.speed.value
                while self.accumulator >= GAIT_DT:
                    self.accumulator -= GAIT_DT
                    steps += 1
            sample = None
            for _ in range(steps):
                sample = generator.update(GAIT_DT, command)
            if sample is None:
                sample = self.last_sample or generator.sample(command)
            self.last_sample = sample
            return gait_frame(self.model, sample, generator.max_speed())

    def draw(self, frame: Frame) -> None:
        follow = self.mode.value == MODES[3] and self.follow.value
        offset = np.array([frame.position[0], frame.position[1], 0.0]) if follow else np.zeros(3)
        with self.server.atomic():
            self.robot.position = frame.position - offset
            self.robot.wxyz = vtf.SO3.from_matrix(frame.rotation).wxyz
            cell = 0.25
            self.grid.position = tuple(-np.mod(offset[:2], cell)) + (0.0,)
            for leg in self.model.legs:
                frames = link_frames(self.model, leg.index, frame.q[leg.index])
                for name in ('coxa', 'femur', 'tibia', 'foot'):
                    handle = self.link_handles[(leg.index, name)]
                    handle.position = frames[name][:3, 3]
                    handle.wxyz = vtf.SO3.from_matrix(frames[name][:3, :3]).wxyz
                marker = self.foot_handles[leg.index]
                marker.position = frame.feet_world[leg.index] - offset + np.array([0.0, 0.0, 0.002])
                marker.visible = bool(frame.contact[leg.index])
            self.com_handle.position = np.array([frame.com_xy[0], frame.com_xy[1], 0.002]) - offset
            self._draw_polygon(frame, offset)
        self.status.content = status_html(frame)
        self.table.content = table_html(self.model, frame.q)

    def _draw_polygon(self, frame: Frame, offset) -> None:
        polygon = frame.polygon
        if len(polygon) >= 2:
            closed = np.c_[polygon - offset[:2], np.full(len(polygon), 0.002)]
            segments = np.stack((closed, np.roll(closed, -1, axis=0)), axis=1)
        else:
            segments = np.zeros((1, 2, 3))
        colour = (20, 150, 60) if frame.margin > 0 else (200, 20, 28)
        # Segment count changes as feet lift, so the node is replaced rather than updated.
        self.server.scene.add_line_segments('/support_polygon', segments, colors=colour, thickness=4.0, thickness_units='screen')
        # The com marker sits under the body, so the margin is also drawn out to the nearest polygon edge.
        com = np.array([frame.com_xy[0], frame.com_xy[1]]) - offset[:2]
        edge = nearest_point_on_polygon(polygon - offset[:2], com) if len(polygon) else com
        line = np.array([[[com[0], com[1], 0.003], [edge[0], edge[1], 0.003]]])
        self.server.scene.add_line_segments('/stability_margin', line, colors=(235, 120, 0), thickness=3.0, thickness_units='screen')

    # ---- loop ----
    def tick(self) -> None:
        if self.dirty or (self.mode.value == MODES[3] and (self.playing or self.pending_steps)):
            self.dirty = False
            self.frame = self.compute_frame()
            self.draw(self.frame)

    def run(self) -> None:
        while self.running:
            started = time.monotonic()
            self.tick()
            time.sleep(max(0.0, TICK - (time.monotonic() - started)))

    def stop(self) -> None:
        self.running = False
        self.server.stop()

