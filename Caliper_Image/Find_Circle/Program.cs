using FindCircle;
using FindStraightEdge;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Xml.Linq;

namespace FindStraightEdge
{
    internal class Program
    {
        static void Main(string[] args)
        {
            string path = "C:\\Users\\Admin\\Downloads\\circleTest.jpg";
            
            for (int i = 545; i <= 555; i++)
            {
                for(int j = 362; j <= 372; j++)
                {
                    Point pCenter = new Point(i, j);
                    double ra = 50;
                    double Ra = 100;
                    ROI_Circle ROI = new ROI_Circle(pCenter, ra, Ra);




                    #region  ///////////////////////////    Option       ////////////////////////////////////////

                    int gap = 15;                   //Khoảng khe hở giữa các đường
                    int thresh = 50;                //Ngưỡng cắt của đường đạo hàm

                    //EdgePolarity polarity = EdgePolarity.AllEdge;

                    //double numline = 12;
                    DirectionFindCircle direction = DirectionFindCircle.OutsidetoInside;
                    OptionsFindCircle Opts = new OptionsFindCircle(gap, thresh, direction);
                    #endregion
                    Stopwatch stopwatch = new Stopwatch();
                    stopwatch.Start();
                    //Gadgets.FindCircle(source, ROI, Opts);
                    SourceImage source = new SourceImage(path);
                    stopwatch.Stop();
                    Console.WriteLine(stopwatch.Elapsed.TotalMilliseconds);
                    source.DisposeSource();

                }

            }
            








            //source.imgOriginal.Dispose();






            Console.ReadKey();
        }
    }
}
