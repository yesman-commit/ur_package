#include <visp3/gui/vpDisplayOpenCV.h>
 #include <visp3/io/vpImageIo.h>
 #include <visp3/io/vpVideoReader.h>
 #include <visp3/vision/vpKeyPoint.h>
 #include <fstream>


 
int main()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
    vpImage<unsigned char> I;
    std::ofstream ofs("/home/tsuchida/output_file/zen.txt");
    
    vpVideoReader reader;
    reader.setFileName("/home/tsuchida/video/video_0.mpeg");
    reader.acquire(I);
    
    const std::string detectorName = "ORB";
    const std::string extractorName = "ORB";
    // Hamming distance must be used with ORB
    const std::string matcherName = "BruteForce-Hamming";
    vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
    vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);
    std::cout << "Reference keypoints=" << keypoint.buildReference(I) << std::endl;
    
    vpImage<unsigned char> Idisp;
    Idisp.resize(I.getHeight(), 2 * I.getWidth());
    Idisp.insert(I, vpImagePoint(0, 0));
    Idisp.insert(I, vpImagePoint(0, I.getWidth()));
    vpDisplayOpenCV d(Idisp, 0, 0, "Matching keypoints with ORB keypoints");
    vpDisplay::display(Idisp);
    vpDisplay::flush(Idisp);
    
    while (!reader.end()) {
        reader.acquire(I);
        Idisp.insert(I, vpImagePoint(0, I.getWidth()));
    
        vpDisplay::display(Idisp);
        vpDisplay::displayLine(Idisp, vpImagePoint(0, I.getWidth()), vpImagePoint(I.getHeight(), I.getWidth()),
                                vpColor::white, 2);
        ofs << "getWidth" << I.getWidth() << ",   " << "getHeight" << I.getHeight() << "  " << std::endl;
    
        unsigned int nbMatch = keypoint.matchPoint(I);
    
        std::cout << "Matches=" << nbMatch << std::endl;
    
        vpImagePoint iPref, iPcur;
        for (unsigned int i = 0; i < nbMatch; i++) {
        keypoint.getMatchedPoints(i, iPref, iPcur);
        ofs << "iPref  " << iPref << "   x  " << iPref.get_i() << "  y  " << iPref.get_j() << "  i " << i << std::endl;
        vpDisplay::displayLine(Idisp, iPref, iPcur + vpImagePoint(0, I.getWidth()), vpColor::green);
        }
        vpDisplay::flush(Idisp);
    
        if (vpDisplay::getClick(Idisp, false))
        break;
    }
 
    vpDisplay::getClick(Idisp);
#endif
 
    return 0;
}