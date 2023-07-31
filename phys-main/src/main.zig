const std = @import("std");
const zlm = @import("zlm");
const Vec3 = zlm.Vec3;

const Rigidbody = struct {
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    mass: f32,
    force: Vec3,

    pub fn update(self: *Rigidbody, delta_time: f32) void {
        self.acceleration = self.force.scale( 1/self.mass);
        self.velocity = self.velocity.add(self.acceleration.scale(delta_time));
        self.position = self.position.add(self.velocity.scale(delta_time));
        self.force = Vec3.zero;
    }

    pub fn setVelocity(self: *Rigidbody, new_velocity: Vec3) void {
        self.velocity = new_velocity;
    }

    pub fn addForce(self: *Rigidbody, new_force: Vec3) void {
        self.force = self.force.add(new_force);
    }

    pub fn applyForce(self: *Rigidbody, new_force: Vec3) void {
        self.addForce(new_force);
    }
};

test "usage" {
    var rb = Rigidbody{.position = zlm.vec3(0,0,0), .velocity = zlm.vec3(0,0,0),
        .acceleration = zlm.vec3(0,0,0),
        .mass = 0,
        .force = zlm.vec3(0,0,0),
    };
    std.debug.print("{any}", .{rb});
}

test "launch" {
    var ball = Rigidbody{
        .position = zlm.vec3(0, 0, 0),
        .velocity = zlm.vec3(0, 0, 0),
        .acceleration = zlm.vec3(0, 0, 0),
        .mass = 1,
        .force = zlm.vec3(0, 0, 0),
    };

    // Apply an initial force to launch the ball
    ball.applyForce(zlm.vec3(10, 100, 0));

    // Drag coefficient
//     const drag_coefficient = 0.5;

    // Linear drag coefficient
    const linear_drag_coefficient = 0.05;

    // Gravitational acceleration
    const gravity = zlm.vec3(0, -9.8, 0);

    // Coefficient of restitution
    const restitution = 0.8;

    // Ground plane position
    const ground_y = 0;

    var old_position = ball.position;

    // Update the state of the ball and print its position every 0.1 seconds
    var time: f32 = 0;
    std.debug.print("\nBall: {}\n", .{ball});
    while (time < 5) : (time += 0.1) {
        // Apply gravitational force
        ball.applyForce(gravity.scale(ball.mass));

        // Apply drag force
//         const speed_squared = ball.velocity.dot(ball.velocity);
//         const drag_magnitude = drag_coefficient * speed_squared;
//         const drag_force = ball.velocity.normalize().scale(-drag_magnitude);
//         ball.applyForce(drag_force);

        // Apply linear drag force
        const drag_force = ball.velocity.scale(-linear_drag_coefficient);
        ball.applyForce(drag_force);

        // Check for collision with ground plane
//         if (ball.position.y <= ground_y) {
//             // Reflect velocity about ground normal and apply restitution
//             ball.velocity.y = -ball.velocity.y * restitution;
//             // Move ball above ground to prevent it from getting stuck
//             ball.position.y = ground_y;
//         }

        // Check for collision with ground plane using CCD
        if (ball.position.y <= ground_y) {
            // Calculate time of collision
            const collision_time = (ground_y - old_position.y) / (ball.position.y - old_position.y);
            // Calculate position at time of collision
            const collision_position = old_position.lerp(ball.position, collision_time);
            // Set position to collision position
            ball.position = collision_position;
            // Reflect velocity about ground normal and apply restitution
            ball.velocity.y = -ball.velocity.y * restitution;
            // Move ball above ground to prevent it from getting stuck
            ball.position.y = ground_y;
        }

        ball.update(0.1);
        old_position = ball.position;
        std.debug.print("Position: {}\n", .{ball.position});
//         std.debug.print("Ball: {}\n", .{ball});
    }
}

fn vec3AddScalar(vec: Vec3, scalar: f32) Vec3 {
    return zlm.vec3(vec.x + scalar, vec.y + scalar, vec.z + scalar);
}

test "vec3AddScalar" {
    const vec: Vec3 = zlm.vec3(1, 2, 3);
    const scalar: f32 = 1;
    const result = vec3AddScalar(vec, scalar);
    std.debug.assert(result.z == zlm.vec3(2, 3, 4).z);
}
