using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Xml.Linq;

namespace Caliper_Gradient
{
    internal class Program
    {
        static void Main(string[] args)
        {
            #region /////////////////////     Ảnh và ROI   ///////////////////////////////////////////////
            // Ảnh
            string path = "C:\\Users\\Admin\\Desktop\\DiLam\\Caliper\\dulieu3\\24.png";
            Mat source = new Mat(path);


            //         { 0, 0, source.Width, 0, source.Width, source.Height, 0, source.Height }
            //        { 826, 682, 613, -92, 156, 32, 369, 808 }
            //        { 0, 0, 100, 0, 100, 100, 0, 100 }
            //        { 694, 396, 828, 396, 828, 645, 694, 645 }
            //        { 828, 396, 828, 645, 694, 645, 694, 396 }
            //        { 702, 412, 689, 636, 823, 644, 835, 420 }
            // { 104 , 419 , 448 , 302 , 353 , 22 , 9 , 139 }
            // { -21 , 215 , 192 , 511 , 537 , 262 , 323 , -33 }
            // { 2, 164, 186, 488, 447, 339, 263, 15 }


            // Toạ độ ROI: theo chiều kim đồng hồ/ ngược kim đồng hồ
            int[] value_ROI = new int[8] { 0, 0, source.Width, 0, source.Width, source.Height, 0, source.Height };

            List<Point> ROI = new List<Point>
            {
                new Point(value_ROI[0], value_ROI[1]),
                new Point(value_ROI[2], value_ROI[3]),
                new Point(value_ROI[4], value_ROI[5]),
                new Point(value_ROI[6], value_ROI[7]),
            };
            #endregion


            #region  ///////////////////////////    Option      ////////////////////////////////////////

            int gap = 15;                   //Khoảng khe hở giữa các đường
            int thresh = 50;                //Ngưỡng cắt của đường đạo hàm
            EdgePolarity polarity = EdgePolarity.AllEdge;
            Orientation orientation = Orientation.Default;

            // Option riêng Calip
            DirectionSearchCalip direction = DirectionSearchCalip.InsideToOutside;          //Hướng bắt điểm: trong ra ngoài, ngoài vào trong
            ModeCalip mode = ModeCalip.BestDistance;                                     //Kiểu bắt điểm: lấy điểm gần/xa nhất, lấy điểm trung vị
            OptionsCalip optionsCalip = new OptionsCalip(gap, thresh, direction, mode, orientation);

            #endregion


            ResultCaliper resultCaliper = Gadgets.Caliper(source, ROI, optionsCalip);


            #region  //////////////////        Vẽ           ///////////////////////////////////////////////
            // Draw base ROI:
            Cv2.Line(source, new Point(value_ROI[0], value_ROI[1]), new Point(value_ROI[2], value_ROI[3]), Scalar.Aqua, 2);
            Cv2.Line(source, new Point(value_ROI[0], value_ROI[1]), new Point(value_ROI[6], value_ROI[7]), Scalar.Aqua, 2);
            Cv2.Line(source, new Point(value_ROI[4], value_ROI[5]), new Point(value_ROI[2], value_ROI[3]), Scalar.Aqua, 2);
            Cv2.Line(source, new Point(value_ROI[4], value_ROI[5]), new Point(value_ROI[6], value_ROI[7]), Scalar.Aqua, 2);
            Cv2.Line(source, resultCaliper.PointCenterStart, resultCaliper.PointCenterEnd, Scalar.Aqua, 2);

            // Draw Grid:
            for (int i = 0; i < resultCaliper.ListPoint_Grid_12.Count; i++)
            {
                Cv2.Line(source, resultCaliper.ListPoint_Grid_12[i], resultCaliper.ListPoint_Grid_34[i], Scalar.Green, 1);
            }

            // Draw list Point in each line of Grid:
            for (int i = 0; i < resultCaliper.ListPointDST_12.Count; i++)
            {
                Cv2.Circle(source, (Point)resultCaliper.ListPointDST_12[i], 2, Scalar.Red, 1);
            }
            for (int i = 0; i < resultCaliper.ListPointDST_34.Count; i++)
            {
                Cv2.Circle(source, (Point)resultCaliper.ListPointDST_34[i], 2, Scalar.Red, 1);
            }

            // Draw line for calip and highlight point Catch:
            if (resultCaliper.List_Point_Catch.Count == 6)
            {
                Cv2.Line(source, (Point)resultCaliper.List_Point_Catch[0], (Point)resultCaliper.List_Point_Catch[1], Scalar.Red, 1);
                Cv2.Line(source, (Point)resultCaliper.List_Point_Catch[2], (Point)resultCaliper.List_Point_Catch[3], Scalar.Red, 1);
                Cv2.Circle(source, (Point)resultCaliper.List_Point_Catch[4], 2, Scalar.Red, 2);
                Cv2.Circle(source, (Point)resultCaliper.List_Point_Catch[5], 2, Scalar.Red, 2);
            }

            // Show the distance: 
            if (resultCaliper.Distance == -1) { Console.WriteLine("Khong tinh duoc khoang cach"); }
            else { Console.WriteLine("Khoang cach tinh duoc la {0}", resultCaliper.Distance); }
            #endregion


            Cv2.ImShow("Original Image", source);
            Cv2.WaitKey(0);
        }
    }
}
