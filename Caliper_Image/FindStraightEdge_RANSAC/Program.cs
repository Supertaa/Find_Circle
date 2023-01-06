using FindStraightEdge;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Xml.Linq;

namespace FindStraightEdge
{
    internal class Program
    {
        static void Main(string[] args)
        {
            #region /////////////////////     Ảnh và ROI   ///////////////////////////////////////////////
            // Ảnh
            string path = "C:\\Users\\Admin\\Desktop\\DiLam\\cffff1e77ab4a2eafba5.jpg";
            Mat source = new Mat(path);


            //         { 0, 0, source.Width, 0, source.Width, source.Height, 0, source.Height }
            //        { 826, 682, 613, -92, 156, 32, 369, 808 }
            //        { 0, 0, 100, 0, 100, 100, 0, 100 }
            //        { 694, 396, 828, 396, 828, 645, 694, 645 }
            //        { 828, 396, 828, 645, 694, 645, 694, 396 }
            //        { 702, 412, 689, 636, 823, 644, 835, 420 }


            // Toạ độ ROI: theo chiều kim đồng hồ/ ngược kim đồng hồ
            int[] value_ROI = new int[8] { 2, 164, 186, 488, 447, 339, 263, 15 };

            List<Point> ROI = new List<Point>
            {
                new Point(value_ROI[0], value_ROI[1]),
                new Point(value_ROI[2], value_ROI[3]),
                new Point(value_ROI[4], value_ROI[5]),
                new Point(value_ROI[6], value_ROI[7]),
            };
            #endregion


            #region  ///////////////////////////    Option       ////////////////////////////////////////

            int gap = 15;                   //Khoảng khe hở giữa các đường
            int thresh = 50;                //Ngưỡng cắt của đường đạo hàm
            EdgePolarity polarity = EdgePolarity.DarkToBright;
            Orientation orientation = Orientation.Default;

            // Option riêng Find Straight Edge
            LookForEdge lookfor = LookForEdge.BestEdge;        // Kiểu cạnh muốn bắt
            double dangle = 3;                                 // Góc lệch cho phép giữa các điểm liên tục (tính theo độ)
            OptionsFindStraightEdge options = new OptionsFindStraightEdge(gap, thresh, polarity, lookfor, dangle, orientation);

            #endregion


            ResultFindStraightEdge resultFindStraightEdge = FindStraightEdge.FindStraightEdges(source, ROI, options);


            #region    /////////////////         Vẽ           //////////////////////////////////////
            // Draw base ROI:
            Cv2.Line(source, new Point(value_ROI[0], value_ROI[1]), new Point(value_ROI[2], value_ROI[3]), Scalar.Aqua, 2);
            Cv2.Line(source, new Point(value_ROI[0], value_ROI[1]), new Point(value_ROI[6], value_ROI[7]), Scalar.Aqua, 2);
            Cv2.Line(source, new Point(value_ROI[4], value_ROI[5]), new Point(value_ROI[2], value_ROI[3]), Scalar.Aqua, 2);
            Cv2.Line(source, new Point(value_ROI[4], value_ROI[5]), new Point(value_ROI[6], value_ROI[7]), Scalar.Aqua, 2);

            // Draw Grid line:
            for (int i = 0; i < resultFindStraightEdge.ListPoint_Grid_14.Count; i++)
            {
                Cv2.Line(source, resultFindStraightEdge.ListPoint_Grid_14[i], resultFindStraightEdge.ListPoint_Grid_23[i], Scalar.Green, 1);
            }

            // Show point in the edge
            for (int i = 0; i < resultFindStraightEdge.ListPoint_DST.Count; i++)
            {
                Cv2.Circle(source, (Point)resultFindStraightEdge.ListPoint_DST[i], 2, Scalar.Red, 1);
            }

            // Draw the straight edge
            if (resultFindStraightEdge.List_PointCatch.Count == 2)
            {
                Cv2.Line(source, resultFindStraightEdge.List_PointCatch[0], resultFindStraightEdge.List_PointCatch[1], Scalar.Red, 1);
                Console.WriteLine("Khoang cach duong thang toi goc O la: {0}", resultFindStraightEdge.Distance_EdgeCatch);
                Console.WriteLine("Goc cua duong thang la: {0}", resultFindStraightEdge.Angle_EdgeCatch);
            }
            else
            {
                Console.WriteLine("Khong tim thay canh nao");
            }
            #endregion


            Cv2.ImShow("Find Straight Edge", source);
            Cv2.WaitKey(0);
        }
    }
}
