#include "mcl.h"

// TODO:
// Distance sensor weighing

void makeParticles(int M) {
    // Create a new list of particles randomly spread throughout a uniform distribution of the field
    for (int i = 0; i < M; i++) {
        particle p {xDist(rng), yDist(rng), 0.0f};
        particles.push_back(p);
    }
}

void motionUpdate(std::vector<float> &pose, float dF) {
    // Use a uniform distribution with simulated "noise" (error) of the rotation sensor
    // Acts as a way to simulate sensor slippage, basically consistent randomness
    std::uniform_real_distribution<float> driveDist(dF - driveNoise * dF, dF + driveNoise * dF);
    std::uniform_real_distribution<float> angleDist(pose[2] - angleNoise, pose[2] + angleNoise);

    for (auto& p : particles) {
        float drive = driveDist(rng) * driveConst;
        float angle = angleDist(rng);

        // Apply the product of the rotation matrix and a forward vector of magnitude "drive"
        p.x += drive * sinf(rad(angle));
        p.y += drive * cosf(rad(angle));
    }
}

std::vector<float> sensorUpdate(lemlib::Chassis *odom, pros::Distance *distX, pros::Distance *distY, pros::Imu *imu, std::vector<float> &init) {
    // Get odometry readings for x and y
    auto pose = odom->getPose();
    float x = pose.x;
    float y = pose.y;
    float theta = wrap((float)imu->get_heading());

    if (distX->get_distance() < 9999) {
        float dx = inch(distX->get_distance() + distXOffset) * cosf(rad(theta)) - init[0];
    }
    
    if (distY->get_distance() < 9999) {
        float dy = inch(distY->get_distance() + distYOffset) * cosf(rad(theta)) - init[0];
    }

    return {x, y, theta};
}

void weighParticles(std::vector<float> &pose) {
    for (auto& p : particles) {
        // If a moved particle exceeds the field bounds, place it back into the field
        if (p.x > bounds[0] || p.x < bounds[1] || p.y > bounds[2] || p.y < bounds[3]) {
            p.x = xDist(rng);
            p.y = yDist(rng);
        }

        float dx = p.x - pose[0];
        float dy = p.y - pose[1];

        // Particle's weight can be calculated using the normal distribution PDF (probability density function)
        p.weight = 1 / std::sqrt(2 * odomNoise * M_PI) * std::exp(-0.5f * (dx * dx + dy * dy) / (odomNoise * odomNoise)); 
    }
}

void resample(int M) {
    std::vector<float> weights;
    for (auto &p : particles) weights.push_back(p.weight);

    std::vector<particle> resampled = particles;
    int j = 0;
    float cum = 0.0f;
    float avg = std::accumulate(weights.begin(), weights.end(), 0.0f) / M;

    std::uniform_real_distribution<float> randomWeight(0.0, avg);
    float rand = randomWeight(rng);

    // Low variance resampling
    for (int i = 0; i < M; i++) {
        while (cum < i * avg + rand) {
            if (j >= M) break;
            cum += weights[j];
            j += 1;
        }

        resampled[i].x = particles[j - 1].x;
        resampled[i].y = particles[j - 1].y;
    }

    particles = resampled;
}

void mcl(lemlib::Chassis *odom, pros::Distance *distX, pros::Distance *distY, pros::Imu *imu, std::vector<float> &init, float dF, int M) {
    // On startup, initialize particles
    if (particles.size() == 0) makeParticles(M);

    // Get initial sensor reading
    std::vector<float> pose = sensorUpdate(odom, distX, distY, imu, init);

    // Move all particles based on forward displacement of the robot
    motionUpdate(pose, dF);

    // Early exit if the robot is not in motion
    // Skips resampling to prevent the particles from converging
    if (std::abs(dF) < odomNoise) {
        float sumX = 0.0f, sumY = 0.0f;
        for (auto &p : particles) {
            sumX += p.x;
            sumY += p.y;
        }

        odom->setPose(sumX / M, sumY / M, pose[2]);
        return;
    }

    // Update sensor readings
    pose = sensorUpdate(odom, distX, distY, imu, init);

    // Calculate probability for each particle
    weighParticles(pose);

    // Keep only the higher weighted particles for better future accuracy
    resample(M);

    // Calculate the final belief by averaging each resampled particle
    float sumX = 0.0f, sumY = 0.0f;
    for (auto &p : particles) {
        sumX += p.x;
        sumY += p.y;
    }

    // Link the particle filter's belief to the robot pose
    odom->setPose(sumX / M, sumY / M, pose[2]);
}
