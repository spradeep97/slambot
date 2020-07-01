#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.01);
    std::normal_distribution<> dist_theta(0.0, 0.001);

    for (auto& p : posterior_) {
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist_theta(generator));
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }

    posterior_.back().pose = pose;

}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        //auto proposal = computeProposalDistribution(posterior_);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    std::vector<particle_t> prior;

    //Random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> U1;
    double M_inv = 1.0 / kNumParticles_;
    //double r = (U1(gen) / (M_inv + 1));
    double r = (((double)rand()) / ((double) RAND_MAX)) * M_inv;
    //std::cout << r << std::endl;
    //Low Variance Sampling:
    double c = posterior_[0].weight;
    int i = 0;
    for (int m = 0; m < kNumParticles_; ++m) {
        double U = r + m * M_inv;
        while (U > c) {
            i++;
            c += posterior_[i].weight;
        }
        //Got the particle to be pushed. Now reset it's weight.
        particle_t p = posterior_[i];
        p.weight = M_inv;
        prior.push_back(p);
    }
    //std::cout << "---" << std::endl;
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for (auto  particle : prior) {
      proposal.push_back(actionModel_.applyAction(particle));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    std::vector<double> likelihood_scores;
    double total_score = 0.0;
    //  For each particle in proposal:
    //    compute it's new weight based on sensor model
    //    push back the particle with new weight into posterior.
    for (const auto& particle : proposal) {
        double score = sensorModel_.likelihood(particle, laser, map);
        likelihood_scores.push_back(score);
        posterior.push_back(particle);
        total_score += score;
    }
    //Normalize the weights.
    int itr = 0;
    for (auto& posterior_particle : posterior) {
        posterior_particle.weight = likelihood_scores[itr] / total_score;
        //std::cout << posterior_particlce.pose.x << " | " << posterior_particlce.pose.y << " | " << posterior_particlce.pose.theta << " | " << std::endl;
        ++itr;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    double x = 0.0, y = 0.0, s_theta = 0.0, c_theta = 0.0;
    for (auto& p : posterior) {
        x += p.weight * p.pose.x;
        y += p.weight * p.pose.y;
        s_theta += p.weight * sin(p.pose.theta);
        c_theta += p.weight * cos(p.pose.theta);
    }
    pose.x = x;
    pose.y = y;
    pose.theta = atan2(s_theta, c_theta);
    pose.utime = posterior[0].pose.utime;
    return pose;
}
