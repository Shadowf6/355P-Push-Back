#pragma once

#include "utils.h"
#include "config.h"
#include "odom.h"
#include "dist.h"

#include "lemlib/chassis/chassis.hpp"

#include <random>

struct particle { 
    float x;
    float y;
    float weight; 
};

class MCL {
    private:
        std::vector<float> pose;
        std::vector<particle> particles;
        std::ranlux24_base rng;

        std::uniform_real_distribution<float> xDist;
        std::uniform_real_distribution<float> yDist;

        Odometry *odom;
        Distance *dist;
        lemlib::Chassis *chassis;        
    
    public:
        MCL(Odometry *odometry, Distance *distance, lemlib::Chassis *chasiss) : 
        odom(odometry), dist(distance), pose({0.0f, 0.0f, 0.0f}), chassis(chasiss), rng(355), xDist(bounds[0], bounds[1]), yDist(bounds[2], bounds[3]) {}

        void makeParticles(int M) {
            for (int i = 0; i < M; i++) {
                particle p {xDist(rng), yDist(rng), 0.0f};
                particles.push_back(p);
            }
        }

        void motionUpdate(float dF) {
            std::uniform_real_distribution<float> driveDist(dF - driveNoise * dF, dF + driveNoise * dF);
            std::uniform_real_distribution<float> angleDist(pose[2] - angleNoise, pose[2] + angleNoise);

            for (auto& p : particles) {
                float drive = driveDist(rng) * driveConst;
                float angle = angleDist(rng);

                p.x += drive * sinf(rad(angle));
                p.y += drive * cosf(rad(angle));
            }
        }

        void sensorUpdate() {
            float x, y, theta;

            auto odomPose = odom->getPose();
            x = odomPose.x;
            y = odomPose.y;
            theta = odomPose.theta;

            pose = {x, y, theta};
        }

        void weighParticles() {
            for (auto& p : particles) {
                if (p.x > bounds[0] || p.x < bounds[1] || p.y > bounds[2] || p.y < bounds[3]) {
                    p.x = xDist(rng);
                    p.y = yDist(rng);
                }

                float dx = p.x - pose[0];
                float dy = p.y - pose[1];

                p.weight = 0.1591549f * std::exp(-0.5f * (dx * dx + dy * dy)); 
            }
        }

        void resample(int M) {
            std::vector<float> weights;
            for (auto &p : particles) weights.push_back(p.weight);

            int j = 0;
            float cum = 0.0f;
            float avg = std::accumulate(weights.begin(), weights.end(), 0.0f) / M;
            
            std::uniform_real_distribution<float> randomWeight(0.0f, avg);
            float rand = randomWeight(rng);

            std::vector<particle> resampled = particles;
            
            for (int i = 0; i < M; i++) {
                while (cum < i * avg + rand) {
                    if (j >= M) break;
                    cum += weights[j];
                    j += 1;
                }

                resampled[i].x = particles[j-1].x;
                resampled[i].y = particles[j-1].y;
            }

            particles = resampled;
        }

        void update(float dF, int M) {
            if (particles.size() == 0) makeParticles(M);

            sensorUpdate();

            motionUpdate(dF);

            if (std::abs(dF) < odomNoise) {
                float sumX = 0.0f, sumY = 0.0f;
                for (auto &p : particles) {
                    sumX += p.x;
                    sumY += p.y;
                }

                chassis->setPose(sumX / M, sumY / M, pose[2]);
            }

            sensorUpdate();

            weighParticles();

            float sumX = 0.0f, sumY = 0.0f;
            for (auto &p : particles) {
                sumX += p.x;
                sumY += p.y;
            }

            chassis->setPose(sumX / M, sumY / M, pose[2]);
        }
};
