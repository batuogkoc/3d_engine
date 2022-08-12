#include <unordered_map>
#include <string>
#include "Camera.hpp"

class Scene{
    public:
        Scene();
        std::unordered_map<std::string, Camera> cameras;    
}