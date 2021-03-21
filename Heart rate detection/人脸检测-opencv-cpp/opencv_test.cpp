#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

using namespace std;
using namespace cv;

/** Function Headers */
void detectAndDisplay(Mat frame);
const char* WIN_NAME = "My Video";

/** Global variables */
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
CascadeClassifier nose_cascade;
CascadeClassifier mouth_cascade;
/** @function main */

int main(int argc, const char** argv)
{
    string myvediosource = "D:\\THU\\����\\������\\�ۺ�\\gitrep\\Comprehensive-practice\\Heart rate detection\\asset\\videos\\demo_2.mp4";//��Ƶ·��
    CommandLineParser parser(argc, argv,
        "{help h||}"
        "{face_cascade|data/haarcascades/haarcascade_frontalface_alt.xml|Path to face cascade.}"
        "{eyes_cascade|data/haarcascades/haarcascade_eye_tree_eyeglasses.xml|Path to eyes cascade.}"
        "{camera|0|Camera device number.}");

    parser.about("\nThis program demonstrates using the cv::CascadeClassifier class to detect objects (Face + eyes) in a video stream.\n"
        "You can use Haar or LBP features.\n\n");
    parser.printMessage();

    String face_cascade_name = samples::findFile(parser.get<String>("face_cascade"));
    String eyes_cascade_name = samples::findFile(parser.get<String>("eyes_cascade"));
    //String nose_cascade_name = samples::findFile(parser.get<String>("nose_cascade"));
    //String mouth_cascade_name = samples::findFile(parser.get<String>("mouth_cascade"));
    String nose_cascade_name = "E:\\OPENCV\\opencv\\sources\\data\\haarcascades\\haarcascade_mcs_nose.xml";
    String mouth_cascade_name = "E:\\OPENCV\\opencv\\sources\\data\\haarcascades\\haarcascade_mcs_mouth.xml";
    cout <<"face_cascade_name = "<< face_cascade_name << endl;

    //-- 1. Load the cascades
    if (!face_cascade.load(face_cascade_name))
    {
        cout << "--(!)Error loading face cascade\n";
        return -1;
    };
    if (!eyes_cascade.load(eyes_cascade_name))
    {
        cout << "--(!)Error loading eyes cascade\n";
        return -1;
    };
    
    if (!nose_cascade.load(nose_cascade_name))
    {
        cout << "--(!)Error loading nose cascade\n";
        return -1;
    };
    if (!mouth_cascade.load(mouth_cascade_name))
    {
        cout << "--(!)Error loading mouth cascade\n";
        return -1;
    };
    
    int camera_device = parser.get<int>("camera");
    VideoCapture capture(myvediosource);//�½�һ����Ƶ��
    //-- 2. Read the video stream
    if (!capture.isOpened())
    {
        cout << "--(!)Error opening video capture\n";
        return -1;
    }
    Size myVideoSize = Size((int)capture.get(CAP_PROP_FRAME_HEIGHT),
        (int)capture.get(CAP_PROP_FRAME_WIDTH));
    cout << "the Vidio's height is:" << myVideoSize.height
        << ", width is:" << myVideoSize.width
        << ", totally frame number is:" << capture.get(CAP_PROP_FRAME_COUNT) << endl;
    //������ʾ���ڵ����ֺʹ�С
    namedWindow(WIN_NAME, WINDOW_AUTOSIZE);
 
    Mat frame;
    int framenum = 1;
    while (capture.read(frame))
    {
        if (frame.empty())
        {
            cout << "--(!) No captured frame -- Break!\n";
            break;
        }
        framenum++;
        //-- 3. Apply the classifier to the frame
        detectAndDisplay(frame);

        if (waitKey(10) == 27)
        {
            break; // escape
        }
    }
    return 0;
}

/** @function detectAndDisplay */

void detectAndDisplay(Mat frame)
{
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);  //BGR ת��Ϊ�Ҷ�ͼ
    equalizeHist(frame_gray, frame_gray);   //ֱ��ͼ���⻯

                                            //-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(60, 60));

    for (size_t i = 0; i < faces.size(); i++)
    {
        Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2); // ������������
        ellipse(frame, center, Size(faces[i].width / 2, faces[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0); // ��Բ
        Mat faceROI = frame_gray(faces[i]);
        std::vector<Rect> eyes;
        std::vector<Rect> noses;
        std::vector<Rect> mouths;

        //-- In each face, detect eyes��nose��mouth
        eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
        nose_cascade.detectMultiScale(faceROI, noses, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
        mouth_cascade.detectMultiScale(faceROI, mouths, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
        
        // eyes
        Point eye_center;
        for (size_t j = 0; j < eyes.size(); j++)
        {
            eye_center = Point(faces[i].x + eyes[j].x + eyes[j].width / 2, faces[i].y + eyes[j].y + eyes[j].height / 2); //�۾�������
            if (eye_center.x > faces[i].x && eye_center.y < faces[i].y + faces[i].height / 1.3) // ȷ���۾������ϣ���ʵǰ�߼��ʱ���Ѿ���֤����һ��
            {
                int radius = cvRound((eyes[j].width + eyes[j].height) * 0.25); //ȡ��
                circle(frame, eye_center, radius, Scalar(255, 0, 0), 4, 8, 0);
            }
        }
        
        // nose
        Point nose_center;
        if (noses.size() > 0)
        {
            nose_center = Point(faces[i].x + noses[0].x + noses[0].width / 2, faces[i].y + noses[0].y + noses[0].height / 2); //���ӵ�����
            if (nose_center.y > eye_center.y) //ȷ���������۾��±�
            {
                rectangle(frame, Point(faces[i].x + noses[0].x, faces[i].y + noses[0].y), Point(faces[i].x + noses[0].x + noses[0].width, faces[i].y + noses[0].y + noses[0].height), Scalar(0, 255, 0), 3, 8, 0); //Point(noses[0].x, noses[0].y), Point(noses[0].x + noses[0].width, noses[0].y + noses[0].height)
                //int radius = cvRound((noses[0].width + noses[0].height)*0.25); //ȡ��
                //circle(frame, nose_center, radius, Scalar(0, 255,0), 4, 8, 0);
                std::cout << "nose!\n";
            }
        }
        // mouth
        if (mouths.size() > 0)
        {
            Point mouth_center(faces[i].x + mouths[0].x + mouths[0].width / 2, faces[i].y + mouths[0].y + mouths[0].height / 2); //��͵�����
            if (mouth_center.y > nose_center.y && mouth_center.y > eye_center.y) // ȷ������ڱ����±�
            {
                int radius = cvRound((mouths[0].width + mouths[0].height) * 0.25); //ȡ��
                circle(frame, mouth_center, radius, Scalar(0, 0, 255), 4, 8, 0);
                std::cout << "mouth!\n";
            }

        }
        
    }
    //-- Show what you got
    imshow(WIN_NAME, frame);
}

