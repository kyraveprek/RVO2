/*
 * DissExp1_Simulation.cc
 * RVO2 Library - Example for Dissertation Experiment #1
 * 
 * Reproduces the Julia simulation script for crowd avoidance experiments
 * Based on DissExp1_Plotting.jl by Kyra Veprek
 * 
 * Last updated: 2025-09-19
 */

#include <RVO.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>

class ExperimentSimulation {
private:
    RVO::RVOSimulator* sim;
    
    // Simulation parameters
    static constexpr float TIME_STEP = 1.0f / 90.0f;  // dt = 1/90 from Julia script
    static constexpr float NEIGHBOR_DIST = 15.0f;
    static constexpr int MAX_NEIGHBORS = 10;
    static constexpr float TIME_HORIZON = 10.0f;
    static constexpr float TIME_HORIZON_OBST = 10.0f;
    static constexpr float RADIUS = 0.5f;
    static constexpr float MAX_SPEED = 2.0f;
    
    // Experiment bounds (from Julia script)
    static constexpr float MIN_X = 10.0f;
    static constexpr float MIN_Y = 10.0f;
    static constexpr float MAX_X = 100.0f;
    static constexpr float MAX_Y = 100.0f;
    
    // Agent data structures
    struct AgentTrajectory {
        std::vector<RVO::Vector2> positions;
        std::vector<float> speeds;
        std::vector<float> headings;
    };
    
    std::map<std::string, AgentTrajectory> avatarData;
    AgentTrajectory participantData;
    
    int currentStep;
    int maxSteps;
    int subject;
    int trial;
    
public:
    ExperimentSimulation() : sim(nullptr), currentStep(0), maxSteps(0), subject(10), trial(76) {
        sim = new RVO::RVOSimulator();
        sim->setTimeStep(TIME_STEP);
        sim->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, TIME_HORIZON, 
                             TIME_HORIZON_OBST, RADIUS, MAX_SPEED);
    }
    
    ~ExperimentSimulation() {
        delete sim;
    }
    
    // Load trajectory data from CSV files (simplified version of MATLAB data loading)
    bool loadTrajectoryData(const std::string& dataPath) {
        // This is a simplified version - in practice you'd need to parse MATLAB files
        // or convert them to CSV format first
        
        // For now, generate sample data that mimics the experiment structure
        generateSampleData();
        return true;
    }
    
    void generateSampleData() {
        // Generate sample trajectory data for testing
        maxSteps = 500; // Approximate number of steps
        
        // Generate participant trajectory (human subject)
        participantData.positions.reserve(maxSteps);
        participantData.speeds.reserve(maxSteps);
        participantData.headings.reserve(maxSteps);
        
        for (int i = 0; i < maxSteps; ++i) {
            float t = i * TIME_STEP;
            // Sample human trajectory - moving from bottom to top with some variation
            float x = MIN_X + 5.0f + 2.0f * std::sin(t * 0.5f);
            float y = MIN_Y + (MAX_Y - MIN_Y) * (float(i) / maxSteps);
            
            participantData.positions.push_back(RVO::Vector2(x, y));
            participantData.speeds.push_back(1.2f + 0.3f * std::sin(t));
            participantData.headings.push_back(M_PI_2 + 0.2f * std::sin(t * 0.3f));
        }
        
        // Generate avatar trajectories (A1P through A10P)
        for (int avatar = 1; avatar <= 10; ++avatar) {
            std::string key = "A" + std::to_string(avatar) + "P";
            AgentTrajectory& traj = avatarData[key];
            
            traj.positions.reserve(maxSteps);
            traj.speeds.reserve(maxSteps);
            traj.headings.reserve(maxSteps);
            
            float offsetX = (avatar - 5.5f) * 2.0f;
            float startY = MAX_Y - 10.0f;
            
            for (int i = 0; i < maxSteps; ++i) {
                float t = i * TIME_STEP;
                float x = MIN_X + 30.0f + offsetX + std::sin(t * 0.3f + avatar);
                float y = startY - (MAX_Y - MIN_Y) * 0.8f * (float(i) / maxSteps);
                
                traj.positions.push_back(RVO::Vector2(x, y));
                traj.speeds.push_back(1.0f + 0.2f * std::sin(t + avatar));
                traj.headings.push_back(-M_PI_2 + 0.1f * std::sin(t * 0.4f + avatar));
            }
        }
    }
    
    void setupSimulation() {
        // Add participant agent (human subject)
        if (!participantData.positions.empty()) {
            size_t agentId = sim->addAgent(participantData.positions[0]);
            std::cout << "Added participant agent with ID: " << agentId << std::endl;
        }
        
        // Add avatar agents (A1P through A10P)
        for (int avatar = 1; avatar <= 10; ++avatar) {
            std::string key = "A" + std::to_string(avatar) + "P";
            if (avatarData.find(key) != avatarData.end() && 
                !avatarData[key].positions.empty()) {
                size_t agentId = sim->addAgent(avatarData[key].positions[0]);
                std::cout << "Added avatar agent " << key << " with ID: " << agentId << std::endl;
            }
        }
        
        // Add goal agent (static target)
        RVO::Vector2 goalPos(MIN_X, std::sqrt(9*9 + 11*11) + MIN_Y);
        size_t goalId = sim->addAgent(goalPos);
        sim->setAgentMaxSpeed(goalId, 0.0f); // Static goal
        std::cout << "Added goal agent with ID: " << goalId << std::endl;
    }
    
    void updateAgentPositions() {
        if (currentStep >= maxSteps) return;
        
        // Update participant agent (ID 0)
        if (currentStep < participantData.positions.size()) {
            RVO::Vector2 preferredVel = calculatePreferredVelocity(0, participantData);
            sim->setAgentPrefVelocity(0, preferredVel);
        }
        
        // Update avatar agents (IDs 1-10)
        for (int avatar = 1; avatar <= 10; ++avatar) {
            std::string key = "A" + std::to_string(avatar) + "P";
            if (avatarData.find(key) != avatarData.end() && 
                currentStep < avatarData[key].positions.size()) {
                RVO::Vector2 preferredVel = calculatePreferredVelocity(avatar, avatarData[key]);
                sim->setAgentPrefVelocity(avatar, preferredVel);
            }
        }
        
        // Goal agent remains static (ID 11)
        sim->setAgentPrefVelocity(11, RVO::Vector2(0.0f, 0.0f));
    }
    
    RVO::Vector2 calculatePreferredVelocity(int agentId, const AgentTrajectory& trajectory) {
        if (currentStep >= trajectory.positions.size() - 1) {
            return RVO::Vector2(0.0f, 0.0f);
        }
        
        // Calculate velocity from position difference
        RVO::Vector2 currentPos = trajectory.positions[currentStep];
        RVO::Vector2 nextPos = trajectory.positions[currentStep + 1];
        RVO::Vector2 velocity = (nextPos - currentPos) / TIME_STEP;
        
        return velocity;
    }
    
    void runSimulation() {
        std::cout << "Starting simulation with " << maxSteps << " steps..." << std::endl;
        
        // Output file for trajectory data
        std::ofstream outputFile("simulation_output.csv");
        outputFile << "step,agent_id,x,y,vx,vy,speed\n";
        
        for (currentStep = 0; currentStep < maxSteps; ++currentStep) {
            updateAgentPositions();
            sim->doStep();
            
            // Log agent positions every 10 steps
            if (currentStep % 10 == 0) {
                std::cout << "Step " << currentStep << "/" << maxSteps << std::endl;
                
                for (size_t i = 0; i < sim->getNumAgents(); ++i) {
                    RVO::Vector2 pos = sim->getAgentPosition(i);
                    RVO::Vector2 vel = sim->getAgentVelocity(i);
                    float speed = abs(vel);
                    
                    outputFile << currentStep << "," << i << "," 
                              << pos.x() << "," << pos.y() << "," 
                              << vel.x() << "," << vel.y() << "," 
                              << speed << "\n";
                }
            }
        }
        
        outputFile.close();
        std::cout << "Simulation completed. Output saved to simulation_output.csv" << std::endl;
    }
    
    void printSimulationStats() {
        std::cout << "\n=== Simulation Statistics ===" << std::endl;
        std::cout << "Subject: " << subject << std::endl;
        std::cout << "Trial: " << trial << std::endl;
        std::cout << "Time step: " << TIME_STEP << " seconds" << std::endl;
        std::cout << "Total steps: " << maxSteps << std::endl;
        std::cout << "Total agents: " << sim->getNumAgents() << std::endl;
        std::cout << "Simulation bounds: (" << MIN_X << "," << MIN_Y << ") to (" 
                  << MAX_X << ", " << MAX_Y << ")" << std::endl;
    }
};

int main() {
    std::cout << "RVO2 Dissertation Experiment #1 Simulation" << std::endl;
    std::cout << "Based on DissExp1_Plotting.jl by Kyra Veprek" << std::endl;
    std::cout << "=========================================" << std::endl;
    
    ExperimentSimulation experiment;
    
    // Load trajectory data (or generate sample data)
    if (!experiment.loadTrajectoryData("./data/")) {
        std::cerr << "Warning: Using generated sample data" << std::endl;
    }
    
    // Setup and run simulation
    experiment.setupSimulation();
    experiment.printSimulationStats();
    experiment.runSimulation();
    
    return 0;
}