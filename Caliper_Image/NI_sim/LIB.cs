using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;
using OpenCvSharp;
using static OpenCvSharp.Stitcher;

namespace Caliper_Gradient
{
    public static class Gadgets
    {
        public static ResultCaliper Caliper(Mat Source, List<Point> ROI, OptionsCalip Opts)
        {
            #region      ////////////           Khởi tạo các điểm và list điểm của ROI         /////////////////////////////////////
            // Các điểm base

            List<Point> ROIProcess = Get_ROIOrientation(ROI, Opts);
            Point point1 = ROIProcess[0];
            Point point2 = ROIProcess[1];
            Point point3 = ROIProcess[2];
            Point point4 = ROIProcess[3];
            Point pointCenterStart = new Point((int)(point1.X + point4.X) / 2, (int)(point1.Y + point4.Y) / 2);
            Point pointCenterEnd = new Point((int)(point2.X + point3.X) / 2, (int)(point2.Y + point3.Y) / 2);

            // Xác định góc của ROI
            double angle = 180 / Math.PI * Math.Atan2(pointCenterEnd.Y - pointCenterStart.Y, pointCenterEnd.X - pointCenterStart.X);   //Góc của ROI
            double angle12 = Math.Atan2(point1.Y - pointCenterStart.Y, point1.X - pointCenterStart.X) * 180 / Math.PI;        //Góc của vùng 12
            double angle34 = Math.Atan2(point4.Y - pointCenterStart.Y, point4.X - pointCenterStart.X) * 180 / Math.PI;       //Góc của vùng 34


            //Tạo list các điểm thuộc 3 đường ngang chính
            //Khởi tạo điểm đầu của list là điểm đầu của các đường ngang
            List<Point> listPoint12 = new List<Point>() { new Point(point1.X, point1.Y) };
            List<Point> listPoint34 = new List<Point>() { new Point(point4.X, point4.Y) };
            List<Point> listPointCenter = new List<Point>() { new Point(pointCenterStart.X, pointCenterStart.Y) };

            // Tính độ dài của ROI
            double widthROI = Math.Sqrt(Math.Pow(pointCenterEnd.X - pointCenterStart.X, 2) + Math.Pow(pointCenterEnd.Y - pointCenterStart.Y, 2));
            // Tìm số lượng point trên các đường ngang chính theo độ dài ROI
            int numPoint = (int)widthROI / Opts.Gap + 1;
            //Console.WriteLine("{0}, {1}", widthROI, numPoint);
            // Add thêm các điểm vào list
            for (int i = 1; i < numPoint; i++)          //Chạy từ 1 bởi điểm đầu tiên 0 đã được tạo
            {
                Point eachPoint12 = RotatePoint(new Point(i * Opts.Gap, 0), new Point(0, 0), angle, point1);
                Point eachPoint34 = RotatePoint(new Point(i * Opts.Gap, 0), new Point(0, 0), angle, point4);
                Point eachPointCenter = RotatePoint(new Point(i * Opts.Gap, 0), new Point(0, 0), angle, pointCenterStart);

                listPoint12.Add(eachPoint12);
                listPoint34.Add(eachPoint34);
                listPointCenter.Add(eachPointCenter);
                //Console.WriteLine("({0}, {1})", ListPointCenter[i].X, ListPointCenter[i].Y);
            }
            #endregion


            #region       ////////////////         Load ảnh và biến đổi ảnh////////////////////////////////////////////////////////
            // Tạo ảnh xám
            Mat ImgGray = new Mat();
            Cv2.CvtColor(Source, ImgGray, ColorConversionCodes.BGR2GRAY);
            Mat ImgBilateral = new Mat();
            Cv2.BilateralFilter(ImgGray, ImgBilateral, 3, 55, 55);

            // Lấy matrix sobel tương ứng theo góc ROI
            var data = SobelMatrix.GetSobelMatrix(angle + 90);                    // Do góc của Roi và đường bắt điểm chênh nhau 90 độ nên phải cộng thêm
            //var data = new int[,] { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };    //Test
            // Gán matrix sobel thành kernel 
            var kernel = new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, data);

            //Tạo và hiển thị ảnh đạo hàm bậc 1 2 theo ma trận kernel
            Mat ImgSobel1 = new Mat();
            Mat ImgSobel2 = new Mat();

            Cv2.Filter2D(ImgBilateral, ImgSobel1, MatType.CV_64FC1, kernel);
            Cv2.Filter2D(ImgSobel1, ImgSobel2, MatType.CV_64FC1, kernel);
            #endregion


            #region /////////////////////      Bắt các điểm            ///////////////////////////////////////////////////////
            // Tạo một list các đường, mỗi đường là 1 dãy pixel từ đầu tới cuối đường lấy điểm đó
            List<List<Point>> ListRangePix12 = new List<List<Point>> { };
            List<List<Point>> ListRangePix34 = new List<List<Point>> { };
            if (Opts.Direction == DirectionSearchCalip.InsideToOutside)
            {
                ListRangePix12 = GetListRangePixel(Source, listPointCenter, listPoint12, angle + 90);     // angle + 90 bởi đường lấy pixel vuông góc với góc ROI
                ListRangePix34 = GetListRangePixel(Source, listPointCenter, listPoint34, angle + 90);
            }
            else if (Opts.Direction == DirectionSearchCalip.OutsideToInside)
            {
                ListRangePix12 = GetListRangePixel(Source, listPoint12, listPointCenter, angle + 90);
                ListRangePix34 = GetListRangePixel(Source, listPoint34, listPointCenter, angle + 90);
            }


            // Lấy hướng xử lý cho các điểm ()
            // Hướng cho biết các pixel bắt được nằm ở đâu so với điểm làm gốc trên đường Center hoặc cạnh 12, 34
            // Khi bắt từ trong ra ngoài lấy hướng theo in-out, ngoài vào trong cạnh làm chuẩn là 12/34 hướng là out-in, chúng ngược nhau
            // Hướng xử lý gán cho dãy điểm 12 in->out, bằng dãy 34 out->in, bằng đối của 12 out->in và 34 in->out
            double direc_x_12_inout, direc_y_12_inout;
            if ((point1.X - pointCenterStart.X) >= 0) { direc_x_12_inout = 1; }
            else { direc_x_12_inout = -1; }
            if ((point1.Y - pointCenterStart.Y) >= 0) { direc_y_12_inout = 1; }
            else { direc_y_12_inout = -1; }


            // Tìm hệ số ax + by + c = 0 của đường tâm ROI:
            double a = pointCenterEnd.Y - pointCenterStart.Y;
            double b = pointCenterStart.X - pointCenterEnd.X;
            double c = pointCenterStart.Y * pointCenterEnd.X - pointCenterStart.X * pointCenterEnd.Y;
            // Tìm khoảng cách với công thức:    d = abs(ax + by +c) / sqrt(a^2 + b^2)


            ////////////////// Tìm các điểm peakpoint và thực hiện tính khoảng cách mỗi điểm tới đường làm chuẩn ///////////////////////

            List<List<double>> list_derivative1_12 = GetListLineInImgSobel(ImgSobel1, ListRangePix12);    //List các đường đạo hàm bậc 1 vùng 12
            List<List<double>> list_derivative2_12 = GetListLineInImgSobel(ImgSobel2, ListRangePix12);    //List các đường đạo hàm bậc 2 vùng 12
            List<Point2d> list_Point12_DST = new List<Point2d> { };                               //Lưu trữ list các điểm bắt được với định dạng Point (int)
            List<double> list_Distance_12 = new List<double> { };          // Lưu trữ khoảng các từng điểm tới đường làm chuẩn
            for (int i = 0; i < list_derivative1_12.Count; i++)
            {
                double x, y;
                double subpix = FindPeakPoint(list_derivative1_12[i], list_derivative2_12[i], Opts);
                if (subpix != -1)
                {
                    if (Opts.Direction == DirectionSearchCalip.InsideToOutside)
                    {
                        if ((angle >= -45 & angle <= 45) | (angle >= 135 & angle <= 225) | (angle >= -225 & angle <= -135))
                        {
                            y = ListRangePix12[i][0].Y + direc_y_12_inout * subpix;                                                   // y = y0 + dy
                            x = ListRangePix12[i][0].X + direc_x_12_inout * subpix / Math.Abs(Math.Tan(angle12 * Math.PI / 180));   // x = x0 + dy*cos/sin
                        }
                        else
                        {
                            x = ListRangePix12[i][0].X + direc_x_12_inout * subpix;                                                   // x = x0 + dx
                            y = ListRangePix12[i][0].Y + direc_y_12_inout * subpix * Math.Abs(Math.Tan(angle12 * Math.PI / 180));   // y = y0 + dx*sin/cos
                        }
                    }
                    else
                    {
                        if ((angle >= -45 & angle <= 45) | (angle >= 135 & angle <= 225) | (angle >= -225 & angle <= -135))
                        {
                            y = ListRangePix12[i][0].Y - direc_y_12_inout * subpix;                                                   // y = y0 + dy
                            x = ListRangePix12[i][0].X - direc_x_12_inout * subpix / Math.Abs(Math.Tan(angle12 * Math.PI / 180));   // x = x0 + dy*cos/sin
                        }
                        else
                        {
                            x = ListRangePix12[i][0].X - direc_x_12_inout * subpix;                                                   // x = x0 + dx
                            y = ListRangePix12[i][0].Y - direc_y_12_inout * subpix * Math.Abs(Math.Tan(angle12 * Math.PI / 180));   // y = y0 + dx*sin/cos
                        }
                    }
                    double d12 = Math.Abs(a * x + b * y + c) / Math.Sqrt(Math.Pow(a, 2) + Math.Pow(b, 2));
                    list_Distance_12.Add(Math.Round(d12, 3));
                    list_Point12_DST.Add(new Point(x, y));
                }
            }


            List<List<double>> list_derivative1_34 = GetListLineInImgSobel(ImgSobel1, ListRangePix34);     //List các đường đạo hàm bậc 1 vùng 34
            List<List<double>> list_derivative2_34 = GetListLineInImgSobel(ImgSobel2, ListRangePix34);     //List các đường đạo hàm bậc 2 vùng 34
            List<Point2d> list_Point34_DST = new List<Point2d> { };                               //Lưu trữ list các điểm bắt được với định dạng Point (int)
            List<double> list_Distance_34 = new List<double> { };          // Lưu trữ khoảng các từng điểm tới đường làm chuẩn
            for (int i = 0; i < list_derivative1_34.Count; i++)
            {
                double x, y;
                double subpix = FindPeakPoint(list_derivative1_34[i], list_derivative2_34[i], Opts);
                if (subpix != -1)
                {
                    if (Opts.Direction == DirectionSearchCalip.InsideToOutside)
                    {
                        if ((angle >= -45 & angle <= 45) | (angle >= 135 & angle <= 180) | (angle >= -180 & angle <= -135))
                        {
                            y = ListRangePix34[i][0].Y - direc_y_12_inout * subpix;
                            x = ListRangePix34[i][0].X - direc_x_12_inout * subpix / Math.Abs(Math.Tan(angle34 * Math.PI / 180));   // x = y*cos/sin
                        }
                        else
                        {
                            x = ListRangePix34[i][0].X - direc_x_12_inout * subpix;
                            y = ListRangePix34[i][0].Y - direc_y_12_inout * subpix * Math.Abs(Math.Tan(angle34 * Math.PI / 180));   // y = x*sin/cos
                        }
                    }
                    else
                    {
                        if ((angle >= -45 & angle <= 45) | (angle >= 135 & angle <= 180) | (angle >= -180 & angle <= -135))
                        {
                            y = ListRangePix34[i][0].Y + direc_y_12_inout * subpix;
                            x = ListRangePix34[i][0].X + direc_x_12_inout * subpix / Math.Abs(Math.Tan(angle34 * Math.PI / 180));   // x = y*cos/sin
                        }
                        else
                        {
                            x = ListRangePix34[i][0].X + direc_x_12_inout * subpix;
                            y = ListRangePix34[i][0].Y + direc_y_12_inout * subpix * Math.Abs(Math.Tan(angle34 * Math.PI / 180));   // y = x*sin/cos
                        }
                    }
                    double d34 = Math.Abs(a * x + b * y + c) / Math.Sqrt(Math.Pow(a, 2) + Math.Pow(b, 2));
                    list_Distance_34.Add(Math.Round(d34, 3));
                    list_Point34_DST.Add(new Point(x, y));
                }
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            #endregion


            #region   ////////////////        Đo khoảng cách, trả về khoảng cách + các điểm bắt được cần vẽ            ///////////

            // Lấy khoảng cách ở 2 bên
            List<double> distance_index_12 = CalculateCalip(list_Distance_12, Opts);      // List chứa (khoảng cách, index) của điểm được chọn để calip
            List<double> distance_index_34 = CalculateCalip(list_Distance_34, Opts);
            double distance = -1;
            List<Point2d> list_Point_Catch = new List<Point2d> { };
            // Nếu khoảng cách cả 2 bên trả về đều đúng =>, Còn không thì dis = -1 (nghĩa là bị sai)
            if (distance_index_12[0] != -1 & distance_index_34[0] != -1)
            {
                // Khoảng cách tính được
                distance = distance_index_12[0] + distance_index_34[0];

                // Các điểm bắt được cuối cùng, p1p2 là điểm dùng để vẽ line đỏ trên vùng 12
                Point2d p1_Catch = RotatePoint(new Point(distance_index_12[0], 0), new Point(0, 0), angle12, pointCenterStart);
                Point2d p2_Catch = RotatePoint(new Point(distance_index_12[0], 0), new Point(0, 0), angle12, pointCenterEnd);
                // Các điểm bắt được cuối cùng, p3p4 là điểm dùng để vẽ line đỏ trên vùng 34
                Point2d p3_Catch = RotatePoint(new Point(distance_index_34[0], 0), new Point(0, 0), angle34, pointCenterStart);
                Point2d p4_Catch = RotatePoint(new Point(distance_index_34[0], 0), new Point(0, 0), angle34, pointCenterEnd);
                // Điểm p5 p6 là 2 điểm được bắt để tính khoảng cách ở 2 bên
                Point2d p5_Catch = list_Point12_DST[(int)distance_index_12[1]];
                Point2d p6_Catch = list_Point34_DST[(int)distance_index_34[1]];

                list_Point_Catch = new List<Point2d> { p1_Catch, p2_Catch, p3_Catch, p4_Catch, p5_Catch, p6_Catch };
            }
            #endregion


            ResultCaliper resultCaliper = new ResultCaliper(pointCenterStart, pointCenterEnd, listPoint12, listPoint34, list_Point12_DST, list_Point34_DST, distance, list_Point_Catch);


            return resultCaliper;
        }


        public static ResultFindStraightEdge FindStraightEdges(Mat Source, List<Point> ROI, OptionsFindStraightEdge Opts)
        {
            #region      ////////////           Khởi tạo các điểm và list điểm của ROI         /////////////////////////////////////
            // Các điểm base
            List<Point> ROIOrientation = Get_ROIOrientation(ROI, Opts);
            Point point1 = ROIOrientation[0];
            Point point2 = ROIOrientation[1];
            Point point3 = ROIOrientation[2];
            Point point4 = ROIOrientation[3];

            // Xác định góc của ROI
            double angle = 180 / Math.PI * Math.Atan2(point2.Y - point1.Y, point2.X - point1.X);    //Góc của ROI
            double angle14 = 180 / Math.PI * Math.Atan2(point4.Y - point1.Y, point4.X - point1.X);  //Góc phụ để tìm điểm


            //Tìm các list điểm để vẽ Grid bắt điểm
            List<Point> listPoint14 = new List<Point>() { new Point(point1.X, point1.Y) };
            List<Point> listPoint23 = new List<Point>() { new Point(point2.X, point2.Y) };

            // Tính độ rộng ROI
            double heightROI = Math.Sqrt(Math.Pow(point4.X - point1.X, 2) + Math.Pow(point4.Y - point1.Y, 2));

            // Tìm số lượng point trên các đường 14 23 để vẽ grid
            int numPoint = (int)heightROI / Opts.Gap + 1;

            // Add thêm các điểm vào list
            for (int i = 1; i < numPoint; i++)          //Chạy từ 1 bởi điểm đầu tiên 0 đã được tạo
            {
                Point eachPoint14 = RotatePoint(new Point(i * Opts.Gap, 0), new Point(0, 0), angle14, point1);
                Point eachPoint23 = RotatePoint(new Point(i * Opts.Gap, 0), new Point(0, 0), angle14, point2);

                listPoint14.Add(eachPoint14);
                listPoint23.Add(eachPoint23);
            }
            #endregion


            #region       ////////////////         Load ảnh và biến đổi ảnh         ///////////////////////////////////////////////////
            // Tạo ảnh xám
            Mat ImgGray = new Mat();
            Cv2.CvtColor(Source, ImgGray, ColorConversionCodes.BGR2GRAY);
            Mat ImgBilateral = new Mat();
            Cv2.BilateralFilter(ImgGray, ImgBilateral, 3, 55, 55);

            // Lấy matrix sobel tương ứng theo góc ROI
            var data = SobelMatrix.GetSobelMatrix(angle);                    // Truyền góc của ROI vào
            //var data = new int[,] { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };    //Test
            // Gán matrix sobel thành kernel
            var kernel = new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, data);

            //Tạo và hiển thị ảnh đạo hàm bậc 1 2 theo ma trận kernel
            Mat ImgSobel1 = new Mat();
            Mat ImgSobel2 = new Mat();

            Cv2.Filter2D(ImgBilateral, ImgSobel1, MatType.CV_64FC1, kernel);
            Cv2.Filter2D(ImgSobel1, ImgSobel2, MatType.CV_64FC1, kernel);
            #endregion


            #region  ////////////////////         Bắt các điểm             ///////////////////////////////////////////////////////////
            List<List<Point>> ListRangePix = GetListRangePixel(Source, listPoint14, listPoint23, angle);


            // Lấy hướng xử lý cho các điểm ()
            // Hướng này để biết pixel bắt được nằm ở đâu so với đường làm gốc 14
            double direc_x, direc_y;
            if ((angle >= -90) & (angle <= 90)) { direc_x = 1; }
            else { direc_x = -1; }
            if ((angle >= 0) & (angle <= 180)) { direc_y = 1; }
            else { direc_y = -1; }


            List<List<double>> list_line_derivative1 = GetListLineInImgSobel(ImgSobel1, ListRangePix);    //List các đường đạo hàm bậc 1
            List<List<double>> list_line_derivative2 = GetListLineInImgSobel(ImgSobel2, ListRangePix);    //List các đường đạo hàm bậc 2

            List<Point2f> list_Point_DST = new List<Point2f> { };      // Lưu trữ các điểm bắt được bằng kiểu float cho độ chính xác subpixel

            for (int i = 0; i < list_line_derivative1.Count; i++)
            {
                double x, y;
                double subpix = FindPeakPoint(list_line_derivative1[i], list_line_derivative2[i], Opts);
                if (subpix != -1)
                {

                    if ((angle >= -45 & angle <= 45) | (angle >= 135 & angle <= 225) | (angle >= -225 & angle <= -135))
                    {
                        x = ListRangePix[i][0].X + direc_x * subpix;                                                   // x = x0 + dx
                        y = ListRangePix[i][0].Y + direc_y * subpix * Math.Abs(Math.Tan(angle * Math.PI / 180));   // y = y0 + dx*sin/cos
                    }
                    else
                    {
                        y = ListRangePix[i][0].Y + direc_y * subpix;                                                   // y = y0 + dy
                        x = ListRangePix[i][0].X + direc_x * subpix / Math.Abs(Math.Tan(angle * Math.PI / 180));   // x = x0 + dy*cos/sin
                    }

                    //list_Point12_DST_subpix.Add(new List<double> { Math.Round(x, 3), Math.Round(y, 3) });
                    list_Point_DST.Add(new Point2f((float)x, (float)y));
                }
            }
            #endregion


            #region ////////////////////////            RANSAC              //////////////////////////////////////////
            int N_pick = 20;     // Số lần lặp
            double maxdis = 0.4;    // Khoảng lệch cho phép
            int max_count = 0;     // Số điểm max của 1 cạnh tìm được

            Point2f pointA = new Point2f();       // 2 điểm bắt được trong cạnh tốt nhất
            Point2f pointB = new Point2f();

            // Sắp xếp ngẫu nhiên
            //Fisher-Yates algorithm
            Random random = new Random();
            void Shuffle<Point2f>(List<Point2f> list)
            {
                int n = list.Count;
                for (int i = 0; i < (n - 1); i++)
                {
                    int r = i + random.Next(n - i);
                    Point2f p = list[r];
                    list[r] = list[i];
                    list[i] = p;
                }
            }

            // Vòng lặp tìm cạnh tốt nhất
            for (int i = 0; i <= N_pick; i++)
            {
                Shuffle<Point2f>(list_Point_DST);
                int count = 0;


                // chọn ngẫu nhiên 2 trong list_point  
                Point2f pointa = list_Point_DST[0];//chọn ngẫu nhiên điểm thứ nhất
                Point2f pointb = list_Point_DST[1];//chọn ngẫu nhiên điểm thứ hai  

                //tính khoảng cách từ tất cả các điểm đến đường thẳng chọn từ 2 điểm ở trên
                // mx + ny + c = 0
                double m = pointb.Y - pointa.Y;
                double n = pointa.X - pointb.X;
                double c = pointb.X * pointa.Y - pointa.X * pointb.Y;


                for (int j = 0; j < list_Point_DST.Count; j++)
                {
                    double X1 = list_Point_DST[j].X;
                    double Y1 = list_Point_DST[j].Y;
                    // Tìm khoảng cách, nếu thoả mãn thì +1 điểm vào cạnh đó
                    double distance = Math.Abs(m * X1 + n * Y1 + c) / Math.Sqrt(m * m + n * n);

                    if (distance < maxdis)
                    {
                        count++;
                    }
                }

                if (count == list_Point_DST.Count)
                {
                    pointA = pointa;
                    pointB = pointb;
                    max_count = list_Point_DST.Count;
                    break;
                }
                else if (count > max_count)
                {
                    pointA = pointa;
                    pointB = pointb;
                    max_count = count;
                }

            }

            Console.WriteLine("So diem bat duoc tren tong so diem {0} / {1}", max_count, list_Point_DST.Count);
            #endregion


            Cv2.Line(Source, (Point)pointA, (Point)pointB, Scalar.Red, 1);
            Cv2.Circle(Source, (Point)pointA, 2, Scalar.Red, 2);
            Cv2.Circle(Source, (Point)pointB, 2, Scalar.Red, 2);





            ResultFindStraightEdge resultFindEdge = new ResultFindStraightEdge(listPoint14, listPoint23, list_Point_DST);

            return resultFindEdge;
        }


        /// <summary>
        /// Hàm sử dụng để thay đổi hướng xử lý của ROI
        /// </summary>
        public static List<Point> Get_ROIOrientation(List<Point> ROI, Options Opts)
        {
            List<Point> ROIProcess = new List<Point>();
            if (Opts.Orientation == Orientation.Default)
            {
                ROIProcess = new List<Point> { ROI[0], ROI[1], ROI[2], ROI[3] };
            }
            else if (Opts.Orientation == Orientation.RotateROI_90)
            {
                ROIProcess = new List<Point> { ROI[1], ROI[2], ROI[3], ROI[0] };
            }
            else if (Opts.Orientation == Orientation.RotateROI_180)
            {
                ROIProcess = new List<Point> { ROI[2], ROI[3], ROI[0], ROI[1] };
            }
            else if (Opts.Orientation == Orientation.RotateROI_270)
            {
                ROIProcess = new List<Point> { ROI[3], ROI[0], ROI[1], ROI[2] };
            }

            return ROIProcess;
        }

        /// <summary>
        /// Hàm sử dụng để xoay các điểm point quanh điểm pointCenter với góc angle, sau đó cộng offset là khoảng cách tới gốc thực
        /// </summary>
        public static Point RotatePoint(Point point, Point pointCenter, double angle, Point offset)
        {
            double x = pointCenter.X + (point.X - pointCenter.X) * Math.Cos(angle * Math.PI / 180)
                - (point.Y - pointCenter.Y) * Math.Sin(angle * Math.PI / 180) + offset.X;
            double y = pointCenter.Y + (point.X - pointCenter.X) * Math.Sin(angle * Math.PI / 180)
                + (point.Y - pointCenter.Y) * Math.Cos(angle * Math.PI / 180) + offset.Y;
            return new Point((int)x, (int)y);
        }


        /// <summary>
        /// Hàm trả về list các đường thẳng
        /// Mỗi đường thẳng này chứa các pixel của đường bắt điểm
        /// </summary>
        public static List<List<Point>> GetListRangePixel(Mat Source, List<Point> ListPointStart, List<Point> ListPointEnd, double angle)
        {
            List<List<Point>> ListRangePix = new List<List<Point>>();

            //Tìm phương trình đường thẳng ax + by + c = 0
            double a = ListPointEnd[0].Y - ListPointStart[0].Y;
            double b = -(ListPointEnd[0].X - ListPointStart[0].X);
            for (int i = 0; i < ListPointStart.Count; i++)
            {
                double c = -ListPointStart[i].X * ListPointEnd[i].Y + ListPointStart[i].Y * ListPointEnd[i].X;
                //Tạo một rangepix, là một list chứa các vị trí pixel
                List<Point> RangePix = new List<Point>();

                if ((angle >= -45 & angle <= 45) || (angle >= 135 & angle <= 225) || (angle >= -255 & angle <= -135))     // Tính pixel nếu đường ngang
                {
                    if (b < 0)
                    {
                        for (int x = ListPointStart[i].X; x <= ListPointEnd[i].X; x++)
                        {
                            double y = -(a * x + c) / b;
                            if (y >= 0 & y < Source.Height & x >= 0 & x < Source.Width)
                            {
                                RangePix.Add(new Point(x, y));
                                //Cv2.Circle(Source, new Point(x, (int)y), 2, Scalar.Red, 1);
                            }
                        }
                    }
                    else
                    {
                        for (int x = ListPointStart[i].X; x >= ListPointEnd[i].X; x--)
                        {
                            double y = -(a * x + c) / b;
                            if (y >= 0 & y < Source.Height & x >= 0 & x < Source.Width)
                            {
                                RangePix.Add(new Point(x, y));
                                //Cv2.Circle(Source, new Point(x, (int)y), 2, Scalar.Red, 1);
                            }
                        }
                    }
                }

                else        // Tính pixel nếu đường dọc
                {
                    if (a > 0)
                    {
                        for (int y = ListPointStart[i].Y; y <= ListPointEnd[i].Y; y++)
                        {
                            double x = -(b * y + c) / a;
                            if (y >= 0 & y < Source.Height & x >= 0 & x < Source.Width)
                            {
                                RangePix.Add(new Point(x, y));
                                //Cv2.Circle(Source, new Point((int)x, y), 2, Scalar.Red, 1);
                            }
                        }
                    }
                    else
                    {
                        for (int y = ListPointStart[i].Y; y >= ListPointEnd[i].Y; y--)
                        {
                            double x = -(b * y + c) / a;
                            if (y >= 0 & y < Source.Height & x >= 0 & x < Source.Width)
                            {
                                RangePix.Add(new Point(x, y));
                                //Cv2.Circle(Source, new Point((int)x, y), 2, Scalar.Red, 1);
                            }
                        }
                    }
                }
                ListRangePix.Add(RangePix);
            }
            return ListRangePix;
        }


        /// <summary>
        /// Hàm trả về list các line
        /// Mỗi line là các giá trị độ xám trên các đường bắt điểm
        /// </summary>
        public static List<List<double>> GetListLineInImgSobel(Mat ImgSobel, List<List<Point>> ListRangePix)
        {
            List<List<double>> ListLineSobel = new List<List<double>>();
            for (int i = 0; i < ListRangePix.Count; i++)
            {
                List<double> LineSobel = new List<double>();
                for (int j = 0; j < ListRangePix[i].Count; j++)
                {
                    if (ListRangePix[i][j].X < ImgSobel.Width & ListRangePix[i][j].Y < ImgSobel.Height)
                    {
                        var ngrayness = ImgSobel.Get<double>(ListRangePix[i][j].Y, ListRangePix[i][j].X);
                        LineSobel.Add(ngrayness);
                    }
                }
                ListLineSobel.Add(LineSobel);
            }
            return ListLineSobel;
        }


        /// <summary>
        /// Hàm tìm các điểm biên cạnh
        /// </summary>
        public static double FindPeakPoint(List<double> line_derivative1, List<double> line_derivative2, Options Opts)
        {
            List<int> edge_1_point = new List<int>();     // List chứa các index thoả mãn
            // Add tất cả index của các điểm thoả mãn lớn hơn Threshold
            for (int i = 0; i < line_derivative1.Count; i++)
            {
                if (Opts.Polarity == EdgePolarity.DarkToBright)
                {
                    double value = line_derivative1[i];
                    if (value > Math.Abs(Opts.Thresh))
                    {
                        edge_1_point.Add(i);
                    }
                }
                else if (Opts.Polarity == EdgePolarity.BrightToDark)
                {
                    double value = line_derivative1[i];
                    if (value < -Math.Abs(Opts.Thresh))
                    {
                        edge_1_point.Add(i);
                    }
                }
                else
                {
                    double value = Math.Abs(line_derivative1[i]);
                    if (value > Math.Abs(Opts.Thresh))
                    {
                        edge_1_point.Add(i);
                    }
                }
            }

            // Xử lý đạo hàm bậc 2, chỉ lấy điểm đầu tiên
            // Lấy các điểm lân cận điểm peakpoint để tìm giá trị subpixel
            double subpix = -1;
            List<double> edge_final = new List<double> { };

            for (int j = 0; j < edge_1_point.Count - 1; j++)
            {
                if ((double)line_derivative2[edge_1_point[j]] * (double)line_derivative2[edge_1_point[j + 1]] < 0)
                {
                    if (edge_1_point[j] >= 1 & edge_1_point[j] < line_derivative1.Count - 2)
                    {
                        edge_final.Add(edge_1_point[j] - 1);
                        edge_final.Add(edge_1_point[j]);
                        edge_final.Add(edge_1_point[j] + 1);
                        edge_final.Add(edge_1_point[j] + 2);
                        break;
                    }
                    else
                    {
                        subpix = edge_1_point[j];
                    }

                }
            }

            //return points_value;
            if (edge_final.Count == 4)
            {
                double[,] A = new double[,]
                {
                    { edge_final[0] * edge_final[0], edge_final[0], 1 },
                    { edge_final[1] * edge_final[1], edge_final[1], 1 },
                    { edge_final[2] * edge_final[2], edge_final[2], 1 }
                };

                double detA = A[0, 0] * (A[1, 1] * A[2, 2] - A[2, 1] * A[1, 2]) -
                              A[0, 1] * (A[1, 0] * A[2, 2] - A[1, 2] * A[2, 0]) +
                              A[0, 2] * (A[1, 0] * A[2, 1] - A[1, 1] * A[2, 0]);

                double invdetA = 1 / detA;

                double[,] invA = new double[,]
                {
                    {(A[1, 1] * A[2, 2] - A[2, 1] * A[1, 2]) * invdetA,
                     (A[0, 2] * A[2, 1] - A[0, 1] * A[2, 2]) * invdetA,
                     (A[0, 1] * A[1, 2] - A[0, 2] * A[1, 1]) * invdetA },
                    {(A[1, 2] * A[2, 0] - A[1, 0] * A[2, 2]) * invdetA,
                     (A[0, 0] * A[2, 2] - A[0, 2] * A[2, 0]) * invdetA,
                     (A[1, 0] * A[0, 2] - A[0, 0] * A[1, 2]) * invdetA },
                    {(A[1, 0] * A[2, 1] - A[2, 0] * A[1, 1]) * invdetA,
                     (A[2, 0] * A[0, 1] - A[0, 0] * A[2, 1]) * invdetA,
                     (A[0, 0] * A[1, 1] - A[1, 0] * A[0, 1]) * invdetA }
                };
                double[] B = new double[]
                {
                    (double)line_derivative1[(int)edge_final[0]],
                    (double)line_derivative1[(int)edge_final[1]],
                    (double)line_derivative1[(int)edge_final[2]]
                };


                //double[] X = invA * B;
                double[] X = new double[]
                {
                    invA[0, 0] * B[0] + invA[0, 1] * B[1] + invA[0, 2] * B[2],
                    invA[1, 0] * B[0] + invA[1, 1] * B[1] + invA[1, 2] * B[2],
                    invA[2, 0] * B[0] + invA[2, 1] * B[1] + invA[2, 2] * B[2]
                };


                double[,] C = new double[,]
                {
                    { edge_final[1]*edge_final[1], edge_final[1], 1 },
                    { edge_final[2]*edge_final[2], edge_final[2], 1 },
                    { edge_final[3]*edge_final[3], edge_final[3], 1 }
                };

                double detC = C[0, 0] * (C[1, 1] * C[2, 2] - C[2, 1] * C[1, 2]) -
                              C[0, 1] * (C[1, 0] * C[2, 2] - C[1, 2] * C[2, 0]) +
                              C[0, 2] * (C[1, 0] * C[2, 1] - C[1, 1] * C[2, 0]);

                double invdetC = 1 / detC;

                double[,] invC = new double[,]
                {
                    {(C[1, 1] * C[2, 2] - C[2, 1] * C[1, 2]) * invdetC,
                     (C[0, 2] * C[2, 1] - C[0, 1] * C[2, 2]) * invdetC,
                     (C[0, 1] * C[1, 2] - C[0, 2] * C[1, 1]) * invdetC },
                    {(C[1, 2] * C[2, 0] - C[1, 0] * C[2, 2]) * invdetC,
                     (C[0, 0] * C[2, 2] - C[0, 2] * C[2, 0]) * invdetC,
                     (C[1, 0] * C[0, 2] - C[0, 0] * C[1, 2]) * invdetC },
                    {(C[1, 0] * C[2, 1] - C[2, 0] * C[1, 1]) * invdetC,
                     (C[2, 0] * C[0, 1] - C[0, 0] * C[2, 1]) * invdetC,
                     (C[0, 0] * C[1, 1] - C[1, 0] * C[0, 1]) * invdetC }
                };

                double[] D = new double[]
                {
                    (double)line_derivative1 [(int)edge_final[1]],
                    (double)line_derivative1 [(int)edge_final[2]],
                    (double)line_derivative1 [(int)edge_final[3]]
                };

                //double[] Y = invC * D;
                double[] Y = new double[]
                {
                    invC[0, 0] * D[0] + invC[0, 1] * D[1] + invC[0, 2] * D[2],
                    invC[1, 0] * D[0] + invC[1, 1] * D[1] + invC[1, 2] * D[2],
                    invC[2, 0] * D[0] + invC[2, 1] * D[1] + invC[2, 2] * D[2]
                };


                double x = -X[1] / (2 * X[0]);
                double y = -Y[1] / (2 * Y[0]);
                if (edge_final[1] <= x & x <= edge_final[2])
                {
                    subpix = x;
                }
                else if (edge_final[1] <= y & y <= edge_final[2])
                {
                    subpix = y;
                }
                else
                {
                    subpix = edge_final[1];
                }
            }
            //Console.WriteLine(subpix);
            return subpix;
        }


        /// <summary>
        /// Hàm riêng của Caliper, dùng để tìm điểm tính calip
        /// Sử dụng cho Calip, trả về khoảng cách và index của điểm lấy calip
        /// </summary>
        public static List<double> CalculateCalip(List<double> list_distance, OptionsCalip Opts)
        {
            double distance = -1;
            double index = -1;
            if (list_distance.Count > 0)
            {
                if (Opts.Direction == DirectionSearchCalip.InsideToOutside & Opts.Mode == ModeCalip.BestDistance)
                {
                    distance = list_distance[0];
                    index = 0;
                    for (int i = 1; i < list_distance.Count; i++)
                    {
                        if (distance > list_distance[i])
                        {
                            distance = list_distance[i];
                            index = i;
                        }
                    }
                }

                else if (Opts.Direction == DirectionSearchCalip.OutsideToInside & Opts.Mode == ModeCalip.BestDistance)
                {
                    distance = list_distance[0];
                    index = 0;
                    for (int i = 1; i < list_distance.Count; i++)
                    {
                        if (distance < list_distance[i])
                        {
                            distance = list_distance[i];
                            index = i;
                        }
                    }
                }

                else if (Opts.Mode == ModeCalip.MedianDistance)
                {
                    List<double> list_distance_sort = new List<double>(list_distance);
                    list_distance_sort.Sort();
                    distance = list_distance_sort[(int)list_distance_sort.Count / 2];

                    for (int i = 0; i < list_distance.Count; i++)
                    {
                        if (distance == list_distance[i])
                        {
                            index = i;
                        }
                    }
                }
            }

            return new List<double> { distance, index };
        }
    }


    /// <summary>
    /// Option để thay đổi các thuộc tính của ứng dụng
    /// Các enum phụ thuộc
    /// </summary>
    #region  //////////////////////      Options       /////////////////////////////////////////////////////////
    public class Options
    {
        // Phần cài đặt chung
        public int Gap = 15;            // Khoảng cách khe hở
        public int Thresh = 100;               // Ngưỡng cắt
        public EdgePolarity Polarity = EdgePolarity.AllEdge;   // Kiểu bắt điểm, từ tối->sáng hoặc ngược lại hoặc cả 2
        public Orientation Orientation = Orientation.Default;              // Xoay hướng xử lý của ROI
    }

    public class OptionsFindStraightEdge : Options
    {
        // Phần cài đặt riêng của Find Straight Edge
        public LookForEdge LookFor = LookForEdge.BestEdge;    // Nhận cạnh best(nhiều điểm) hoặc cạnh first(gần nhất), hoặc cạnh LongestEdge(dài nhất)
        public double AllowedDeflectionAngle = 3;         // Góc lệch cho phép giữa các điểm

        public OptionsFindStraightEdge(int gap, int thresh, EdgePolarity polarity, LookForEdge lookfor)
        {
            Gap = gap;
            Thresh = thresh;
            Polarity = polarity;
            LookFor = lookfor;
        }

        public OptionsFindStraightEdge(int gap, int thresh, EdgePolarity polarity, LookForEdge lookfor, double allowed_dangle)
        {
            Gap = gap;
            Thresh = thresh;
            Polarity = polarity;
            LookFor = lookfor;
            AllowedDeflectionAngle = allowed_dangle;
        }

        public OptionsFindStraightEdge(int gap, int thresh, EdgePolarity polarity, LookForEdge lookfor, double allowed_dangle, Orientation orientation)
        {
            Gap = gap;
            Thresh = thresh;
            Polarity = polarity;
            LookFor = lookfor;
            AllowedDeflectionAngle = allowed_dangle;
            Orientation = orientation;
        }


    }

    public class OptionsCalip : Options
    {
        // Phần cài đặt riêng của Caliper
        public DirectionSearchCalip Direction = DirectionSearchCalip.InsideToOutside;   // Dùng với Calip, hướng trong ngoài, ngoài trong
        public ModeCalip Mode = ModeCalip.BestDistance;  // Dùng với Calip, dùng để chọn kiểu đo, lớn/nhỏ nhất hoặc giá trị trung vị

        public OptionsCalip(int gap, int thresh, DirectionSearchCalip direction, ModeCalip mode)
        {
            Gap = gap;
            Thresh = thresh;
            Direction = direction;
            Mode = mode;
        }

        public OptionsCalip(int gap, int thresh, DirectionSearchCalip direction, ModeCalip mode, Orientation orientation)
        {
            Gap = gap;
            Thresh = thresh;
            Direction = direction;
            Mode = mode;
            Orientation = orientation;
        }
    }


    /// <summary>
    /// Hướng tìm kiếm điểm: trong ra ngoài, ngoài vào trong, dùng cho Caliper
    /// </summary>
    public enum DirectionSearchCalip
    {
        InsideToOutside,
        OutsideToInside
    }


    /// <summary>
    /// Mode có 2 loại: Điểm bắt tốt nhất là khoảng cách nhỏ/lớn nhất tuỳ theo clip trong hay ngoài, khoảng cách trung vị giữa chúng
    /// </summary>
    public enum ModeCalip
    {
        BestDistance,
        MedianDistance
    }


    /// <summary>
    /// Cách bắt cạnh: trắng sang đen, đen sang trắng, cả 2
    /// </summary>
    public enum EdgePolarity
    {
        DarkToBright,
        BrightToDark,
        AllEdge
    }


    /// <summary>
    /// Kiểu cạnh muốn bắt, cạnh gần nhất so với đường làm chuẩn, cạnh nhiều điểm nhất và cạnh dài nhất
    /// </summary>
    public enum LookForEdge
    {
        FirstEdge,
        BestEdge
    }


    /// <summary>
    /// Xoay hướng xử lý của ROI
    /// </summary>
    public enum Orientation
    {
        Default,
        RotateROI_90,
        RotateROI_180,
        RotateROI_270
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    #endregion


    /// <summary>
    /// Hàm lấy ma trận kernel phục vụ Sobel ảnh theo hướng của ROI
    /// </summary>
    public static class SobelMatrix
    {
        static private int[,] Matrix_LeftToRight = new int[,]     //Ma trix trái -> phải
        {
            { -1, 0, 1 },
            { -2, 0, 2 },
            { -1, 0, 1 }
        };

        static private int[,] Matrix_RightToLeft = new int[,]     //Ma trix phải -> trái
        {
            { 1, 0, -1 },
            { 2, 0, -2 },
            { 1, 0, -1 }
        };

        static private int[,] Matrix_TopToBottom = new int[,]        //Matrix trên -> dưới
        {
            { -1, -2, -1 },
            {  0,  0,  0 },
            {  1,  2,  1 },
        };

        static private int[,] Matrix_BottomToTop = new int[,]        //Matrix trên -> dưới
        {
            {  1,  2,  1 },
            {  0,  0,  0 },
            { -1, -2, -1 },
        };

        static private int[,] MatrixCross_LeftTopToRightBot = new int[,]    //Ma trix hướng trái trên -> phải dưới
        {
            { -2, -1,  0 },
            { -1,  0,  1 },
            {  0,  1,  2 },
        };

        static private int[,] MatrixCross_RightTopToLeftBot = new int[,]    //Ma trix hướng phải trên -> trái dưới
        {
            {  0, -1, -2 },
            {  1,  0, -1 },
            {  2,  1,  0 },
        };

        static private int[,] MatrixCross_LeftBotToRightTop = new int[,]    //Ma trix hướng trái dưới -> phải trên
        {
            {  0,  1,  2 },
            { -1,  0,  1 },
            { -2, -1,  0 },
        };

        static private int[,] MatrixCross_RightBotToLeftTop = new int[,]    //Ma trix hướng phải dưới -> trái trên
        {
            {  2,  1,  0 },
            {  1,  0, -1 },
            {  0, -1, -2 },
        };

        public static int[,] GetSobelMatrix(double angle)
        {
            do { angle = angle + 360; }
            while (angle < -180);

            do { angle = angle - 360; }
            while (angle > 180);

            int[,] matrix = new int[3, 3];
            if ((angle >= -22.5 & angle <= 22.5))
            {
                matrix = Matrix_LeftToRight;
            }
            else if ((angle >= 157.5 & angle <= 180) | (angle >= -180 & angle <= -157.5))
            {
                matrix = Matrix_RightToLeft;
            }
            else if (angle >= 67.5 & angle <= 112.5)
            {
                matrix = Matrix_TopToBottom;
            }
            else if (angle >= -112.5 & angle <= -67.5)
            {
                matrix = Matrix_BottomToTop;
            }
            else if (angle > 22.5 & angle < 67.5)
            {
                matrix = MatrixCross_LeftTopToRightBot;
            }
            else if (angle > -67.5 & angle < -22.5)
            {
                matrix = MatrixCross_LeftBotToRightTop;
            }
            else if (angle > 112.5 & angle < 157.5)
            {
                matrix = MatrixCross_RightTopToLeftBot;
            }
            else if (angle > -157.5 & angle < -112.5)
            {
                matrix = MatrixCross_RightBotToLeftTop;
            }

            return matrix;
        }
    }


    /// <summary>
    /// Kiểu trả về của hàm Caliper
    /// </summary>
    public class ResultCaliper
    {
        public Point PointCenterStart = new Point();
        public Point PointCenterEnd = new Point();

        public List<Point> ListPoint_Grid_12 = new List<Point> { };
        public List<Point> ListPoint_Grid_34 = new List<Point> { };

        public List<Point2d> ListPointDST_12 = new List<Point2d> { };
        public List<Point2d> ListPointDST_34 = new List<Point2d> { };

        public double Distance;
        public List<Point2d> List_Point_Catch = new List<Point2d> { };

        public ResultCaliper(Point pCenterStart, Point pCenterEnd, List<Point> list_p12, List<Point> list_p34,
            List<Point2d> list_p_DST_12, List<Point2d> list_p_DST_34, double distance, List<Point2d> list_pCatch)
        {
            PointCenterStart = pCenterStart;
            PointCenterEnd = pCenterEnd;

            ListPoint_Grid_12 = list_p12;
            ListPoint_Grid_34 = list_p34;

            ListPointDST_12 = list_p_DST_12;
            ListPointDST_34 = list_p_DST_34;

            Distance = distance;
            List_Point_Catch = list_pCatch;
        }
    }


    /// <summary>
    /// Kiểu trả về của hàm Find Straight Edge
    /// </summary>
    public class ResultFindStraightEdge
    {
        public List<Point> ListPoint_Grid_14 = new List<Point> { };
        public List<Point> ListPoint_Grid_23 = new List<Point> { };

        public List<Point2f> ListPoint_DST = new List<Point2f> { };

        public List<Point> List_PointCatch = new List<Point> { };

        public double Distance_EdgeCatch;
        public double Angle_EdgeCatch;
        public ResultFindStraightEdge(List<Point> listPoint_Grid_14, List<Point> listPoint_Grid_23, List<Point2f> listPoint_DST)
        {
            ListPoint_Grid_14 = listPoint_Grid_14;
            ListPoint_Grid_23 = listPoint_Grid_23;
            ListPoint_DST = listPoint_DST;
        }
    }
}
