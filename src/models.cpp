#include "models.h"
#include <iostream>
#include <stdio.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

namespace simulation {

//
// Small angle pendulum
//
SmallAnglePendulumModel::SmallAnglePendulumModel() { reset(); }

void SmallAnglePendulumModel::reset() {
  t = 0.f;
  theta = theta0;
}

void SmallAnglePendulumModel::step(float dt) {
  t += dt;
  theta = smallAnglePendulum(t, theta0, armLength, mass, gravity);
}

float smallAnglePendulum(float t, float theta0, float l, float mass,
                         float graivty) {
  using std::cos;
  using std::sqrt;
  return theta0 * cos(sqrt(graivty / l) * t);
}

vec3f pendulumPosition(float theta, float l) {
  //   /
  //	/
  // o
  using std::cos;
  using std::sin;

  float y = -l * cos(theta);
  float x = l * sin(theta);
  return {x, y, 0.f};
}

//
// Double Pendulum
//
DoublePendulumModel::DoublePendulumModel() { reset(); }

void DoublePendulumModel::reset() {
  theta0 = 5.f;
  theta1 = 10.f;
  p0 = 0.f;
  p1 = 0.f;
}

void DoublePendulumModel::step(float dt) {

  float cosDeltaTheta = std::cos(theta0 - theta1);
  float sinDeltaTheta = std::sin(theta0 - theta1);
  float denom = (m * l * l) * (16.f - 9.f * cosDeltaTheta * cosDeltaTheta);

  // velocities
  float v0 = 6.f * (2.f * p0 - 3.f * cosDeltaTheta * p1) / denom;
  float v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;

  // forces
  float f0 = -(0.5f * m * l * l) *
             (v0 * v1 * sinDeltaTheta + 3.f * (g / l) * std::sin(theta0));
  float f1 = -(0.5f * m * l * l) *
             (-v0 * v1 * sinDeltaTheta + (g / l) * std::sin(theta1));

  // update kinematic/dynamic quantites using Euler integration
  // update momentum
  p0 = p0 + f0 * dt;
  p1 = p1 + f1 * dt;

  // Semi-implicit Euler
  // would use the updated momemnta/velocities for the position updates
  v0 = 6.f * (2.f * p0 - 3.f * cosDeltaTheta * p1) / denom;
  v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;

  // update (angular) positions
  theta0 = theta0 + v0 * dt;
  theta1 = theta1 + v1 * dt;
}

vec3f DoublePendulumModel::mass0Position() const {
  return l * vec3f(std::sin(theta0), -std::cos(theta0), 0.f);
}

vec3f DoublePendulumModel::mass1Position() const {
  using std::cos;
  using std::sin;
  return l * vec3f(sin(theta0) + sin(theta1), -cos(theta0) - cos(theta1), 0.f);
}

//
// Particle Model
//
ParticleModel::ParticleModel() { reset(); }

void ParticleModel::reset() {
  particles.clear();
  // setup
  for (int i = -5; i <= 5; ++i) {
    particles.push_back(Particle({i, i, 0.f}, {-i, 3.f * i, 0.f}));
  }
}

void ParticleModel::step(float dt) {
  for (int iter = 0; iter < 16; ++iter) {
    // do collisions
    for (auto &p : particles) {
      if (length(p.x) > bounds) {
        auto n = normalize(p.x);
        p.v = glm::reflect(p.v, n);
      }
    }

    // move particles
    for (auto &p : particles) {
      // forward Euler
      p.x += p.v * dt;
    }
  }
}

MassSpring1Model::MassSpring1Model() { reset(); }

void MassSpring1Model::reset() {
    particles.clear();
    particles.push_back(ParticleMass(0.f, vec3f{ 0.f }));
    particles.push_back(ParticleMass(1.f, vec3f{ 0.f, -5.f, 0.f }, vec3f{5.f, 0.f, 0.f}));
    createSprings(springs, 5.f);
}

void MassSpring1Model::step(float dt) {
    for (Spring &s : springs) {
        ParticleMass pi = particles[s.particle1];
        ParticleMass pj = particles[s.particle2];

        vec3f fijs = -(s.stiffness) * (glm::length(pi.pos - pj.pos) - s.restLength)*((pi.pos - pj.pos) / glm::length((pi.pos - pj.pos)));
        vec3f d = glm::normalize(pi.pos - pj.pos);
        
        vec3f fijd = -(s.damping) * (glm::dot((pi.velo - pj.velo), d) / (glm::dot(d, d))) * d;

        particles[s.particle1].nforce += fijs + fijd;

        particles[s.particle2].nforce += - fijs - fijd;
    }
    for (ParticleMass& pi : particles) {
        //vec3f fcoll = calculateCollisionForce(pi);
        vec3f fgravity = vec3f{ 0, - pi.mass * g, 0 };
        pi.nforce = pi.nforce + fgravity;// +fcoll;
    }
    for (ParticleMass& pi : particles) {
        if (pi.mass > 0.f) {
            pi.velo = pi.velo + (pi.nforce / pi.mass) * dt;
            pi.pos = pi.pos + pi.velo * dt;
        }
        pi.nforce = vec3f{ 0.f };
    }
}

void MassSpring1Model::createSprings(std::vector<Spring>& springs, float lmin) {
    springs.clear();
    for (int i = 0; i < particles.size(); ++i) {
        for (int j = i + 1; j < particles.size(); ++j) {
            float d = glm::distance(particles[i].pos, particles[j].pos);
            if (d <= lmin) {
                springs.push_back(Spring(d, i, j, 1.f, 0.2f));
            }
        }
    }
}

MassSpring2Model::MassSpring2Model() { reset(); }

void MassSpring2Model::reset() {
    particles.clear();
    particles.push_back(ParticleMass(0.f, vec3f{ 0.f, 25.f, 0.f }));
    for (int i = 1; i <15; ++i) {
        particles.push_back(ParticleMass(1.f, vec3f{ 2.f * i, 25.f, 0.f }));
    }
    createSprings(springs, 2.f);
}

void MassSpring2Model::step(float dt) {
    for (Spring& s : springs) {
        ParticleMass pi = particles[s.particle1];
        ParticleMass pj = particles[s.particle2];

        vec3f fijs = -(s.stiffness) * (glm::length(pi.pos - pj.pos) - s.restLength) * ((pi.pos - pj.pos) / glm::length((pi.pos - pj.pos)));
        vec3f d = glm::normalize(pi.pos - pj.pos);

        vec3f fijd = -(s.damping) * (glm::dot((pi.velo - pj.velo), d) / (glm::dot(d, d))) * d;

        particles[s.particle1].nforce += fijs + fijd;

        particles[s.particle2].nforce += -fijs - fijd;
    }
    for (ParticleMass& pi : particles) {
        //vec3f fcoll = calculateCollisionForce(pi);
        vec3f fgravity = vec3f{ 0, -pi.mass * g, 0 };
        pi.nforce = pi.nforce + fgravity;// +fcoll;
    }
    for (ParticleMass& pi : particles) {
        if (pi.mass > 0.f) {
            pi.velo = pi.velo + (pi.nforce / pi.mass) * dt;
            pi.pos = pi.pos + pi.velo * dt;
        }
        pi.nforce = vec3f{ 0.f };
    }
}

void MassSpring2Model::createSprings(std::vector<Spring>& springs, float lmin) {
    springs.clear();
    for (int i = 0; i < particles.size(); ++i) {
        for (int j = i + 1; j < particles.size(); ++j) {
            float d = glm::distance(particles[i].pos, particles[j].pos);
            if (d <= lmin) {
                springs.push_back(Spring(d, i, j, 200.f, 3.f));
            }
        }
    }
}

MassSpring3Model::MassSpring3Model() { reset(); }

void MassSpring3Model::reset() {
    particles.clear();
    glm::mat4 R = glm::rotate(glm::f32mat4(1.f), glm::radians(12.5f), glm::normalize(vec3f{ 1.f, 1.f, 1.f }));
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            for (int k = 0; k < 10; ++k) {
                particles.push_back(ParticleMass(1.f, vec3f(R * glm::vec4(vec3f{ 2.f * i -10, 2.f * j, 2.f * k }, 1.f))));
            }
        }
    }
   
    createSprings(springs, sqrt(8) + 0.001f);
}

vec3f calculateCollisionForce(ParticleMass& pi) {
    if (pi.pos.y >= -10.f)
        return vec3f{ 0.f };

    float stiff = 50.f;
    float damp = 10.f;

    ParticleMass pj = ParticleMass(0.f, vec3f{ pi.pos.x, 0.f, pi.pos.z });

    vec3f fijs = -(stiff) * (glm::length(pi.pos - pj.pos) - 0.f) * ((pi.pos - pj.pos) / glm::length((pi.pos - pj.pos)));
    vec3f d = glm::normalize(pi.pos - pj.pos);

    vec3f fijd = -(damp) * (glm::dot((pi.velo - pj.velo), d) / (glm::dot(d, d))) * d;

    return fijs + fijd;
}

void MassSpring3Model::step(float dt) {
    for (Spring& s : springs) {
        ParticleMass pi = particles[s.particle1];
        ParticleMass pj = particles[s.particle2];

        vec3f fijs = -(s.stiffness) * (glm::length(pi.pos - pj.pos) - s.restLength) * ((pi.pos - pj.pos) / glm::length((pi.pos - pj.pos)));
        vec3f d = glm::normalize(pi.pos - pj.pos);

        vec3f fijd = -(s.damping) * (glm::dot((pi.velo - pj.velo), d) / (glm::dot(d, d))) * d;

        particles[s.particle1].nforce += fijs + fijd;

        particles[s.particle2].nforce += -fijs - fijd;
    }
    for (ParticleMass& pi : particles) {
        vec3f fcoll = calculateCollisionForce(pi);
        vec3f fgravity = vec3f{ 0, -pi.mass * g, 0 };
        pi.nforce = pi.nforce + fgravity + fcoll;
    }
    for (ParticleMass& pi : particles) {
        if (pi.mass > 0.f) {
            pi.velo = pi.velo + (pi.nforce / pi.mass) * dt;
            pi.pos = pi.pos + pi.velo * dt;
        }
        pi.nforce = vec3f{ 0.f };
    }
}

void MassSpring3Model::createSprings(std::vector<Spring>& springs, float lmin) {
    springs.clear();
    for (int i = 0; i < particles.size() - 1; ++i) {
        for (int j = i + 1; j < particles.size(); ++j) {
            float d = glm::distance(particles[i].pos, particles[j].pos);
            if (d <= lmin) {
                springs.push_back(Spring(d, i, j, 750.f, 10.f));
            }
        }
    }
}

MassSpring4Model::MassSpring4Model() { reset(); }

void MassSpring4Model::reset() {
    particles.clear();
    particles.push_back(ParticleMass(0.f, vec3f{ -20.f, 10.f, 0.f }));
    for (int i = 1; i < 19; ++i) {
        particles.push_back(ParticleMass(0.5f, vec3f{ 2.f * i - 20, 10.f, 0.f }));
    }
    particles.push_back(ParticleMass(0.f, vec3f{ 2.f * 19 - 20, 10.f, 0.f }));
    for (int j = 0; j < 20; ++j) {
        for (int i = 1; i < 20; ++i) {
            particles.push_back(ParticleMass(0.5f, vec3f{ 2.f * j -20, 10.f, 2.f * i }));
        }
    }

    createSprings(springs, sqrt(8)+0.001f);
}

void MassSpring4Model::step(float dt) {
    for (Spring& s : springs) {
        ParticleMass pi = particles[s.particle1];
        ParticleMass pj = particles[s.particle2];

        vec3f fijs = -(s.stiffness) * (glm::length(pi.pos - pj.pos) - s.restLength) * ((pi.pos - pj.pos) / glm::length((pi.pos - pj.pos)));
        vec3f d = glm::normalize(pi.pos - pj.pos);

        vec3f fijd = -(s.damping) * (glm::dot((pi.velo - pj.velo), d) / (glm::dot(d, d))) * d;

        particles[s.particle1].nforce += fijs + fijd;

        particles[s.particle2].nforce += -fijs - fijd;
    }
    for (ParticleMass& pi : particles) {
        //vec3f fcoll = calculateCollisionForce(pi);
        vec3f fgravity = vec3f{ 0, -pi.mass * g, 0 };
        pi.nforce = pi.nforce + fgravity;// +fcoll;
    }
    for (ParticleMass& pi : particles) {
        if (pi.mass > 0.f) {
            pi.velo = pi.velo + (pi.nforce / pi.mass) * dt;
            pi.pos = pi.pos + pi.velo * dt;
        }
        pi.nforce = vec3f{ 0.f };
    }
}

void MassSpring4Model::createSprings(std::vector<Spring>& springs, float lmin) {
    springs.clear();
    for (int i = 0; i < particles.size() - 1; ++i) {
        for (int j = i + 1; j < particles.size(); ++j) {
            float d = glm::distance(particles[i].pos, particles[j].pos);
            if (d <= lmin) {
                springs.push_back(Spring(d, i, j, 600.f, 3.f));
            }
        }
    }
}

} // namespace simulation
