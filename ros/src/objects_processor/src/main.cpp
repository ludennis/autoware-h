#include <objects_processor.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "objects_processor");

    ObjectsProcessor::ObjectsProcessor objectsProcessor;
	objectsProcessor.Run();

    return 0;
}
