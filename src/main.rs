//#![deny(warnings)]
use crate::debug_plugin::DebugPlugin;
use crate::physics::*;
use bevy::color::palettes::basic::*;
use bevy::color::palettes::css::*;
use bevy::math::vec3;
use bevy::prelude::*;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_trackball::prelude::*;
use std::f32::consts::*;

pub mod camera_config;
pub mod debug_plugin;
pub mod physics;

fn main() {
    let mut app = App::new();

    app.add_plugins(DefaultPlugins)
        .add_plugins(DebugPlugin)
        .add_plugins(TrackballPlugin)
        .add_plugins(InfiniteGridPlugin)
        .add_plugins(physics::PhysicsPlugin)
        .add_systems(Startup, setup_camera)
        .add_systems(Startup, setup_car)
        .add_systems(Update, movements)
        .add_systems(Update, update_driving_wheel)
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1000.0,
        })
        .add_event::<Movement>()
        .run();
}

#[derive(Component)]
struct Car {
    width: f32,
    length: f32,
    height: f32,
}

#[derive(Component)]
struct Wheel {
    max_angle: f32,
    angle: f32,
    min_angle: f32,
}

#[derive(Component)]
struct SteeringAxle;

#[derive(Component)]
struct DrivingWheel;

#[derive(Event)]
enum Movement {
    Forward,
    Backward,
    SteerRight,
    SteerLeft,
}

fn setup_camera(mut commands: Commands) {
    let eye = vec3(0.0, 10.0, 20.0);
    let target = vec3(0.0, 0.0, 0.0);
    let up = Vec3::Y;
    commands.spawn((
        Camera3dBundle { ..default() },
        camera_config::trackball_config(),
        TrackballCamera::look_at(target, eye, up),
    ));
}

fn setup_car(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(InfiniteGridBundle::default());
    let ground_size = 10_000.0;
    let ground_thickness = 1.0;
    let ground_collider = Collider::cuboid(ground_size, ground_thickness, ground_size);

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(ground_size, ground_thickness, ground_size)),
            material: materials.add(Color::Srgba(GRAY)),
            transform: Transform {
                translation: vec3(0.0, -1.0, 0.0),
                ..default()
            },
            ..default()
        },
        physics::rigid_body_fixed(),
        ground_collider,
        //Friction::new(1.0),
    ));

    let obstacle_size = 60.0;
    let obstacle = Cuboid::new(obstacle_size, obstacle_size, obstacle_size);
    let obstacle_collider = Collider::cuboid(
        obstacle_size / 2.0,
        obstacle_size / 2.0,
        obstacle_size / 2.0,
    );
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(obstacle),
            material: materials.add(Color::Srgba(ORANGE)),
            transform: Transform {
                translation: vec3(10.0, -20.0, -40.0),
                rotation: Quat::from_rotation_y(-30f32.to_radians())
                    * Quat::from_rotation_x(30f32.to_radians()),
                ..default()
            },
            ..default()
        },
        //Friction::new(1.0),
        physics::rigid_body_fixed(),
        obstacle_collider,
    ));

    const CAR_GROUP: Group = Group::GROUP_1;

    let car_scale = 2.0;

    let car_length = 0.9 * car_scale;
    let car_width = 0.85 * car_scale;
    let car_height = 0.3 * car_scale;

    let wheel_height = car_height * -Vec3::Y;
    let car_front = car_length * Vec3::Z;
    let car_back = car_length * -Vec3::Z;
    let car_left = car_width * -Vec3::X;
    let car_right = car_width * Vec3::X;

    let tire_front_left = car_front * 0.95 + car_left * 0.85 + wheel_height * 0.75;
    let tire_front_right = car_front * 0.95 + car_right * 0.85 + wheel_height * 0.75;
    let tire_rear_left = car_back * 0.95 + car_left * 0.85 + wheel_height * 0.75;
    let tire_rear_right = car_back * 0.95 + car_right * 0.85 + wheel_height * 0.75;

    let wheel_params = [
        tire_front_left,
        tire_front_right,
        tire_rear_left,
        tire_rear_right,
    ];

    let suspension_height = 0.5;
    let max_steering_angle = 35.0f32.to_radians();
    let wheel_radius = 0.28 * car_scale;
    let car_position = vec3(0.0, wheel_radius + suspension_height, 0.0);

    let body_position_in_car_space = car_position;

    let car_body = Cuboid {
        half_size: vec3(car_width, car_height, car_length),
    };

    let car_collider = Collider::cuboid(car_width, car_height, car_length);

    let car_entity = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(car_body),
                material: materials.add(Color::Srgba(GREEN)),
                transform: Transform {
                    translation: vec3(0.0, 10.0, 0.0),
                    ..default()
                },
                ..default()
            },
            physics::rigid_body_dynamic(),
            physics::mass(2_000.0),
            car_collider,
            Car {
                width: car_width,
                height: car_height,
                length: car_length,
            },
        ))
        .id();

    for (wheel_id, wheel_pos_in_car_space) in wheel_params.into_iter().enumerate() {
        let is_front = wheel_id >= 2;
        let is_left = wheel_id == 0 || wheel_id == 2;
        let is_right = wheel_id == 1 || wheel_id == 3;
        let wheel_center = car_position + wheel_pos_in_car_space;

        let wheel_thickness = wheel_radius / 2.0;
        let axle_mass_props = MassProperties {
            mass: 100.0,
            ..default()
        };

        let axle_mesh = Cuboid::new(2.0, 0.1, 0.1);
        let axle_co = Collider::cuboid(2.0, 0.1, 0.1);

        let mut locked_axes = JointAxesMask::LIN_X
            | JointAxesMask::LIN_Z
            | JointAxesMask::ANG_X
            | JointAxesMask::ANG_Z;

        if !is_front {
            locked_axes |= JointAxesMask::ANG_Y;
        }

        let suspension_attachment_in_body_space =
            wheel_pos_in_car_space - body_position_in_car_space;

        let mut suspension_joint = GenericJointBuilder::new(locked_axes)
            .limits(JointAxis::LinY, [0.0, suspension_height])
            .motor_position(JointAxis::LinY, 0.0, 1.0e4, 1.0e3)
            .local_anchor1(suspension_attachment_in_body_space);

        if is_front {
            suspension_joint =
                suspension_joint.limits(JointAxis::AngY, [-max_steering_angle, max_steering_angle]);
        }

        let axle_entity = commands
            .spawn((
                PbrBundle {
                    mesh: meshes.add(axle_mesh),
                    material: materials.add(Color::Srgba(YELLOW)),
                    transform: Transform {
                        translation: wheel_center,
                        ..default()
                    },
                    ..default()
                },
                // dont put the axle collider here
                physics::rigid_body_dynamic(),
                AdditionalMassProperties::MassProperties(axle_mass_props),
                ImpulseJoint::new(
                    car_entity,
                    TypedJoint::GenericJoint(suspension_joint.build()),
                ),
            ))
            .id();

        // ISSUE: There is no way for Collider::cylinder to be rotated to orient correctly, so
        // using ball collider only
        // TODO: we want to rotate the wheel collider 90 degree at z axis first
        // before spawning the entity
        let wheel_co = Collider::cylinder(wheel_thickness, wheel_radius);
        // The only way to orient the mesh is rotating the mesh prior to spawning
        let mut wheel_mesh: Mesh = Cylinder {
            half_height: wheel_thickness,
            radius: wheel_radius,
        }
        .into();

        // work around to orient the collider to rotate 90 degree on the Z axis
        // https://github.com/dimforge/bevy_rapier/issues/569#issuecomment-2246429119
        let mut revolute = RevoluteJointBuilder::new(Vec3::X).build();
        revolute
            .data
            .set_local_basis2(Quat::from_rotation_z(FRAC_PI_2));

        let wheel_entity = commands
            .spawn((
                PbrBundle {
                    mesh: meshes.add(wheel_mesh),
                    material: if is_front {
                        materials.add(Color::Srgba(GREEN))
                    } else {
                        materials.add(Color::Srgba(RED))
                    },
                    transform: Transform {
                        translation: wheel_center,
                        ..default()
                    },
                    ..default()
                },
                wheel_co,
                // don't let this rest, otherwise if has rested, we can not set the motore velocity
                // anymore
                Sleeping::disabled(),
                physics::rigid_body_dynamic(),
                ImpulseJoint::new(axle_entity, revolute),
                Friction::new(1.0),
                physics::mass(20.0),
            ))
            .id();

        if is_front {
            commands.entity(wheel_entity).insert(DrivingWheel);
            commands.entity(axle_entity).insert(SteeringAxle);
        }
    }

    // some dynamic obstacle
    let num = 16;
    let rad = 0.1;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = rad;

    let cube = Cuboid::new(rad * 2.0, rad * 2.0, rad * 2.0);
    let cube_collider = Collider::cuboid(rad, rad, rad);
    let cube_color = materials.add(Color::Srgba(PURPLE));
    let cube_handle = meshes.add(cube);

    for j in 0usize..1 {
        for k in 0usize..4 {
            for i in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift + centerx;
                let loc = vec3(x, y, z);
                commands.spawn((
                    PbrBundle {
                        mesh: cube_handle.clone(),
                        material: cube_color.clone(),
                        transform: Transform {
                            translation: loc,
                            ..default()
                        },
                        ..default()
                    },
                    physics::rigid_body_dynamic(),
                    cube_collider.clone(),
                ));
            }
        }
    }
}

fn movements(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut movement_writer: EventWriter<Movement>,
) {
    if keys.pressed(KeyCode::KeyW) {
        println!("forward");
        movement_writer.send(Movement::Forward);
    }
    if keys.pressed(KeyCode::KeyS) {
        println!("backward");
        movement_writer.send(Movement::Backward);
    }

    if keys.pressed(KeyCode::KeyA) {
        movement_writer.send(Movement::SteerLeft);
    }
    if keys.pressed(KeyCode::KeyD) {
        movement_writer.send(Movement::SteerRight);
    }
}

fn update_driving_wheel(
    time: Res<Time>,
    mut gizmos: Gizmos,
    mut steering_axle: Query<(&mut Transform, &mut ImpulseJoint), With<SteeringAxle>>,
    mut driving_wheel: Query<
        (&mut Transform, &mut ImpulseJoint),
        (With<DrivingWheel>, Without<SteeringAxle>),
    >,
    mut events: EventReader<Movement>,
) {
    let mut thrust = 0.0;
    let boost = 1.0;
    let drive_strength = 1.0;
    let max_steering_angle = 35.0f32.to_radians();
    let mut steering = 0.0;

    for movement in events.read() {
        match movement {
            Movement::Forward => {
                thrust = -drive_strength;
            }
            Movement::Backward => {
                thrust = drive_strength;
            }
            Movement::SteerLeft => {
                steering = 1.0;
            }
            Movement::SteerRight => {
                steering = -1.0;
            }
            _ => (),
        }
    }

    // ISSUE: it is not really rotating the axle, instead the body is rotated in a weird way
    for (i, (axle_transform, mut impulse_joint)) in steering_axle.iter_mut().enumerate() {
        println!("steering right: {i}");
        impulse_joint.data.as_mut().set_motor_position(
            JointAxis::AngY,
            max_steering_angle * steering,
            1.0e4,
            1.0e3,
        );
    }

    let differential_strength = 0.5;
    let sideways_shift = (max_steering_angle * steering).sin() * differential_strength;
    let speed_diff = if sideways_shift > 0.0 {
        f32::hypot(1.0, sideways_shift)
    } else {
        1.0 / f32::hypot(1.0, sideways_shift)
    };

    let ms = [1.0 / speed_diff, speed_diff];

    for (i, (wheel_transform, mut impulse_joint)) in driving_wheel.iter_mut().enumerate() {
        impulse_joint.data.as_mut().set_motor_velocity(
            JointAxis::AngX,
            30.0 * thrust * ms[i] * boost,
            1.0e2,
        );
    }
}
