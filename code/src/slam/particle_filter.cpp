#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <common/angle_functions.hpp>

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

    std:: random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.01);

    for(auto& p : posterior_) {
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist(generator));
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

    double M = kNumParticles_;
    double r = (((double) rand()) / ((double) RAND_MAX)) * (1.0 / M);
    double c = posterior_[0].weight;
    int i = 0;
    double U;

    for (int m = 1; m <= M; m++) {
        // calculate the next sample point
        U = r + (m - 1) * (1.0 / M);
        // find the first weight that puts us past the sample point
        while (U > c) {
            i ++;
            c += posterior_[i].weight;
        }
        particle_t p = posterior_[i];
        p.weight = 1.0 / M;
        prior.push_back(p);
    }
    return prior;
}

std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for(auto& p : prior) {
        proposal.push_back(actionModel_.applyAction(p));
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
    std::vector<double> particleWeight;

    posterior = proposal;
    for(auto& p : posterior) {
        particleWeight.push_back(sensorModel_.likelihood(p, laser, map)); //find score (= weight) for each particle in posterior
    }

    double sumWeights = 0;
    for (int i = 0; i < particleWeight.size(); i++) {
        sumWeights += particleWeight[i];                //find sum of scores
    }
    for (int i = 0; i < particleWeight.size(); i++) {
        posterior[i].weight = particleWeight[i] / sumWeights; //set posterior particles' weight as the new normalized score
    }

    return posterior;
}

pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    // pose_xyt_t weightedMean;
    std::vector<double> weightedMean_x;
    std::vector<double> weightedMean_y;
    double sinSum = 0.0;
    double cosSum = 0.0;
    double xSum = 0.0;
    double ySum = 0.0;

    for(auto& p : posterior) {
        weightedMean_x.push_back(p.pose.x * p.weight);
        xSum += p.pose.x * p.weight;

        weightedMean_y.push_back(p.pose.y * p.weight);
        ySum += p.pose.y * p.weight;

        sinSum += p.weight * sin(p.pose.theta);
        cosSum += p.weight * cos(p.pose.theta);
    }

    // printf("x: %d\n", weightedMean_x.size());
    // printf("y: %d\n", weightedMean_y.size());

    // std::sort(weightedMean_x.begin(), weightedMean_x.end());
    // std::reverse(weightedMean_x.begin(), weightedMean_x.end());
    // weightedMean_x.erase(weightedMean_x.begin() + 10, weightedMean_x.end());
    //
    // std::sort(weightedMean_y.begin(), weightedMean_y.end());
    // std::reverse(weightedMean_y.begin(), weightedMean_y.end());
    // weightedMean_y.erase(weightedMean_y.begin() + 10, weightedMean_y.end());
    //
    // double sumWeightedMeans_x = 0.0;
    // double sumWeightedMeans_y = 0.0;
    // for (int i = 0; i < weightedMean_x.size(); i++) {
    //     sumWeightedMeans_x += weightedMean_x[i];
    //     sumWeightedMeans_y += weightedMean_y[i];
    // }
    // pose.x = weightedMean.x / xSum;
    // pose.y = weightedMean.y / ySum;
    
    pose.x = xSum;
    pose.y = ySum;
    pose.theta = atan2(sinSum, cosSum);

    return pose;
}
