#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace std; 

class Arena {

    public:

        int width;
        int height;
        int obstacles;
        float total_area;
        std::string image_path;
        int detected_markers[4];

        Arena(const std::string & path) {

            width = 1000;
            height =1000;
            obstacles = 0;
            total_area = 0;
            image_path = path;
        
        }

        void text_file(){
            std::ofstream file("obstacles.txt");
            if (file.is_open()) {
                file << "Aruco ID: [";
                for (int i = 0; i < 4; ++i) {
                    file << detected_markers[i];
                    if (i < 3) {
                        file << ", "; 
                    }
                }
                file << "]";
                file << "\nObstacles: " << obstacles << "\n";
                file << "Area: " << total_area << "\n";
                file.close();
            } else {
                cerr << "Failed to open file for writing.\n";
            }
        }
};

int main() {

    std::string image_path;
    image_path = 'task1c_image.jpg';
    Arena arena(image_path);
    arena.text_file();
    return 0;

}