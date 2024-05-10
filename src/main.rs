#![warn(clippy::all, clippy::pedantic)]

mod colours;

use crate::colours::{BALL_COLOURS, GUNMETAL};
use macroquad::{
    color::Color,
    input::{is_key_released, KeyCode},
    miniquad::date,
    rand::{self as macroquad_rand, srand},
    shapes::draw_circle,
    window::{clear_background, next_frame, Conf},
};
use rand::{rngs::StdRng, Rng, SeedableRng};
use rand_distr::Standard;
use rapier2d::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    },
    geometry::{
        BroadPhaseMultiSap, ColliderBuilder, ColliderSet, CollisionEvent, CollisionEventFlags,
        NarrowPhase,
    },
    math::Isometry,
    na::{vector, DVector, Vector2},
    pipeline::{ActiveEvents, ChannelEventCollector, PhysicsPipeline, QueryPipeline},
    prelude::nalgebra,
};
use uom::{
    si::{
        f32::{Length, Velocity},
        length, velocity,
    },
    unit,
};

#[macro_use]
extern crate uom;

const WINDOW_WIDTH: f32 = 1366.0;
const WINDOW_HEIGHT: f32 = 768.0;
const BALL_RADIUS: f32 = 0.6;

unit! {
    system: uom::si;
    quantity: uom::si::length;

    // 1 metre is 50 px
    @pixel: 0.02; "px", "pixel", "pixels";
}

#[derive(Debug)]
struct Ball {
    radius: Length,
    position: Vector2<Length>,
    physics_handle: Option<RigidBodyHandle>,
    colour: Color,
}

impl Default for Ball {
    fn default() -> Ball {
        let window_width = Length::new::<pixel>(WINDOW_WIDTH);
        let window_height = Length::new::<pixel>(WINDOW_HEIGHT);
        Ball {
            radius: Length::new::<length::meter>(BALL_RADIUS),
            position: vector![
                window_width / 2.0,
                Length::new::<length::meter>(2.0 * BALL_RADIUS) - window_height
            ],
            physics_handle: None,
            colour: BALL_COLOURS[macroquad_rand::gen_range(0, BALL_COLOURS.len())],
        }
    }
}

impl Ball {
    fn physics_handle(&mut self, physics_handle: RigidBodyHandle) -> &mut Ball {
        self.physics_handle = Some(physics_handle);
        self
    }
}

fn conf() -> Conf {
    #[allow(clippy::cast_possible_truncation)]
    Conf {
        window_title: String::from("Macroquad Rapier Bubbles with Units of Measurement"),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        high_dpi: true,
        ..Default::default()
    }
}

fn create_physics_for_ball(
    ball: &Ball,
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    normal_distribution: &mut StdRng,
) -> RigidBodyHandle {
    // Standard generates values in the [0,1) range
    let pseudo_random_value: f32 = normal_distribution.sample(Standard);
    let x_velocity: Velocity =
        Velocity::new::<velocity::meter_per_second>((2.0 * pseudo_random_value) - 1.0);
    let y_velocity: Velocity = Velocity::new::<velocity::meter_per_second>(1.0);
    let linear_velocity = vector![x_velocity.value, y_velocity.value];
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![ball.position.x.value, ball.position.y.value])
        .linvel(linear_velocity)
        .build();
    let collider = ColliderBuilder::ball(BALL_RADIUS)
        .restitution(0.0)
        .density(0.001)
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, rigid_body_set);
    ball_body_handle
}

fn create_ceiling(ceiling_width: Length, max_balls: u32, collider_set: &mut ColliderSet) {
    let collider_half_thickness = Length::new::<length::meter>(0.05);
    let nsubdivs: usize = (max_balls * 2)
        .try_into()
        .expect("Expected fewer subdivisions");
    let heights = DVector::from_fn(nsubdivs + 1, |i, _| if i % 2 == 0 { -1.2 } else { 0.0 });
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let collider = ColliderBuilder::heightfield(
        heights,
        vector![ceiling_width.value, collider_half_thickness.value],
    )
    .translation(vector![
        0.5 * window_width.get::<length::meter>(),
        -1.0 * collider_half_thickness.value
    ])
    .friction(1.0)
    .restitution(0.0)
    .build();
    collider_set.insert(collider);
}

fn create_ground(collider_set: &mut ColliderSet) {
    let collider_half_thickness = Length::new::<length::meter>(0.05);
    let window_height = Length::new::<pixel>(WINDOW_HEIGHT);
    let collider = ColliderBuilder::cuboid(100.0, collider_half_thickness.value)
        .translation(vector![
            0.0,
            (-window_height - collider_half_thickness).get::<length::meter>()
        ])
        .sensor(true)
        .build();
    collider_set.insert(collider);
}

fn create_side_walls(gap: Length, collider_set: &mut ColliderSet) {
    // left wall
    let collider_half_thickness = Length::new::<length::meter>(0.05);
    let window_height = Length::new::<pixel>(WINDOW_HEIGHT);
    let collider = ColliderBuilder::cuboid(
        0.5 * window_height.get::<length::meter>(),
        collider_half_thickness.value,
    )
    .position(Isometry::new(
        vector![
            (gap - collider_half_thickness).get::<length::meter>(),
            (window_height / -2.0).get::<length::meter>()
        ],
        std::f32::consts::FRAC_PI_2,
    ))
    .build();
    collider_set.insert(collider);

    // right wall
    let collider_half_thickness = Length::new::<length::meter>(0.05);
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let collider = ColliderBuilder::cuboid(
        (window_height / 2.0).get::<length::meter>(),
        collider_half_thickness.value,
    )
    .position(Isometry::new(
        vector![
            (window_width + collider_half_thickness - gap).get::<length::meter>(),
            (window_height / -2.0).get::<length::meter>()
        ],
        3.0 * std::f32::consts::FRAC_PI_2,
    ))
    .build();
    collider_set.insert(collider);
}

fn draw_balls(balls: &[Ball]) {
    for ball in balls {
        let Ball {
            colour,
            position,
            radius,
            ..
        } = ball;
        draw_circle(
            position.x.get::<pixel>(),
            -position.y.get::<pixel>(),
            radius.get::<pixel>(),
            *colour,
        );
    }
}

fn update_balls(balls: &mut [Ball], rigid_body_set: &RigidBodySet) {
    for ball in balls {
        if let Some(handle) = ball.physics_handle {
            let ball_body = &rigid_body_set[handle];
            ball.position = vector![
                Length::new::<length::meter>(ball_body.translation().x),
                Length::new::<length::meter>(ball_body.translation().y)
            ];
        }
    }
}

/// Maximum number of ball which can fit across the window
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation
)]
fn get_max_balls() -> u32 {
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let ball_radius = Length::new::<length::meter>(BALL_RADIUS);

    (window_width / (2.0 * ball_radius)).value.floor() as u32
}

#[macroquad::main(conf)]
async fn main() {
    // seed macroquad random number generator
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    {
        srand(date::now().floor() as u64);
    }

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    // Create the ground
    create_ground(&mut collider_set);

    // Create the ceiling
    let max_balls: u32 = get_max_balls();

    let ceiling_width: Length;
    #[allow(clippy::cast_precision_loss)]
    {
        ceiling_width = Length::new::<length::meter>(max_balls as f32 * BALL_RADIUS * 2.0);
    }
    create_ceiling(ceiling_width, max_balls, &mut collider_set);

    // gap at left and right edge of window
    let window_width = Length::new::<pixel>(WINDOW_WIDTH);
    let gap: Length = 0.5 * (window_width - ceiling_width);

    // Create the left and right wall
    create_side_walls(gap, &mut collider_set);

    // Create ball
    let mut normal_distribution = StdRng::from_entropy();
    let mut new_ball = Ball::default();
    let ball_body_handle = create_physics_for_ball(
        &new_ball,
        &mut rigid_body_set,
        &mut collider_set,
        &mut normal_distribution,
    );
    new_ball.physics_handle(ball_body_handle);
    let mut balls: Vec<Ball> = Vec::with_capacity(256);
    balls.push(new_ball);

    // Create other structures necessary for the simulation
    // positive gravity indicates it is applied upwards (reversed)
    let gravity = vector![0.0, 1.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhaseMultiSap::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let (collision_send, collision_recv) = crossbeam::channel::unbounded();
    let (contact_force_send, _contact_force_recv) = crossbeam::channel::unbounded();
    let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);

    let mut paused = false;

    // run the game loop, stepping the simulation once per frame.
    loop {
        if is_key_released(KeyCode::Escape) {
            break;
        }

        clear_background(GUNMETAL);
        draw_balls(&balls);

        if !paused {
            physics_pipeline.step(
                &gravity,
                &integration_parameters,
                &mut island_manager,
                &mut broad_phase,
                &mut narrow_phase,
                &mut rigid_body_set,
                &mut collider_set,
                &mut impulse_joint_set,
                &mut multibody_joint_set,
                &mut ccd_solver,
                Some(&mut query_pipeline),
                &physics_hooks,
                &event_handler,
            );

            // update ball positions (used for drawing)
            update_balls(&mut balls, &rigid_body_set);

            // wait for existing balls to settle before spawning a new one
            if island_manager.active_dynamic_bodies().is_empty() {
                let mut new_ball = Ball::default();
                let ball_body_handle = create_physics_for_ball(
                    &new_ball,
                    &mut rigid_body_set,
                    &mut collider_set,
                    &mut normal_distribution,
                );
                new_ball.physics_handle(ball_body_handle);
                balls.push(new_ball);
            }

            // end simulation if a ball touches the ground
            while let Ok(collision_event) = collision_recv.try_recv() {
                if let CollisionEvent::Started(
                    _collider_handle_1,
                    _collider_handle_2,
                    CollisionEventFlags::SENSOR,
                ) = collision_event
                {
                    paused = true;
                };
            }
        }
        next_frame().await;
    }
}
