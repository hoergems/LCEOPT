#include "oppt/plugin/Plugin.hpp"
#include "Types.hpp"
#include "ContTagInitialBeliefOptions.hpp"
#include "TagState.hpp"

namespace oppt
{
class ContTagInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    ContTagInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~ContTagInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<ContTagInitialBeliefOptions>(optionsFile);
        auto options = static_cast<const ContTagInitialBeliefOptions *>(options_.get());
        //map_.loadFromFile(options->mapPath);
        std::istringstream is("#####...##\n#####...##\n#####...##\n..........\n..........");
        map_.loadFromStream(is);

        initRobotStartingPositions_();
        initTargetStartingPositions_();
        
        std::string positionFile = "startingPosition_" + options->logFilePostfix + ".txt";        
        // Check if file exists
        if (oppt::resources::FileExists(positionFile)) {
            LOGGING("Get starting position from file...");
            std::ifstream f;
            f.open(positionFile);
            FloatType a, b;
            f >> a >> b;
            f.close();
            startPosition_.x() = a;
            startPosition_.y() = b;
            std::remove(positionFile.c_str());
            LOGGING("Done")
        } else {
            std::ofstream f;
            f.open(positionFile);
            f << startPosition_.x() << " " << startPosition_.y() << "\n";
            f.close();
        }  

        LOGGING("STARTING POSITION: " + std::to_string(startPosition_.x()) + ", " + std::to_string(startPosition_.y()));      
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();        
        auto targetPosition = possibleTargetStartFields_[targetStartDistrOne_->operator()(*randomEngine)];
        targetPosition.x() += targetStartDistrTwo_->operator()(*randomEngine);
        targetPosition.y() += targetStartDistrTwo_->operator()(*randomEngine);
        return RobotStateSharedPtr(new TagState(startPosition_, 0.0, targetPosition, false));
    }

private:
    std::vector<Position> possibleTargetStartFields_;

    IntDistributionPtr targetStartDistrOne_ = nullptr;

    UniformRealDistributionPtr targetStartDistrTwo_ = nullptr;

    Position startPosition_;

    Map map_;

private:
    void initRobotStartingPositions_() {
        FloatType sumX = 0;
        FloatType sumY = 0;
        std::vector<Position> possibleStartPositions;
        for (size_t y = 0; y < map_.sizeY(); y++ ) {
            for (size_t x = 0; x < map_.sizeX(); x++ ) {
                if (map_(x, y) == '.') {
                    possibleStartPositions.emplace_back(Position(x + 0.5, y + 0.5));
                }
            }
        }

        if (possibleStartPositions.size() == 0) {
            std::cout << "No start coordinates found." << std::endl;
            assert(false);
        }

        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();

        std::uniform_int_distribution<int> d(0, possibleStartPositions.size() - 1);
        startPosition_ = possibleStartPositions[d(*randomEngine)];

        std::uniform_real_distribution<FloatType> distribution(-0.5, 0.5);
        startPosition_.x() += distribution(*randomEngine);
        startPosition_.y() += distribution(*randomEngine);
        //std::cout << "Selected robot start position: " << startPosition_ << std::endl;
    }

    void initTargetStartingPositions_() {
        possibleTargetStartFields_.clear();        
        for (size_t y = 0; y < map_.sizeY(); y++ ) {
            for (size_t x = 0; x < map_.sizeX(); x++ ) {
                if (map_(x, y) == '.') {
                    possibleTargetStartFields_.emplace_back(Position(x + 0.5, y + 0.5));
                }
            }
        }

        if (possibleTargetStartFields_.empty() )
            ERROR("No possible human start field coordinates found.");

        targetStartDistrOne_ = IntDistributionPtr(new IntDistribution(0, possibleTargetStartFields_.size() - 1));
        targetStartDistrTwo_ = UniformRealDistributionPtr(new UniformRealDistribution(-0.5, 0.5));

    }
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(ContTagInitialBeliefPlugin)

}

