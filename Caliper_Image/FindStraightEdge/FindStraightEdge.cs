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

namespace FindStraightEdge
{
    public static class FindStraightEdge
    {
        public static ResultFindStraightEdge FindStraightEdges(Mat Source, List<Point> ROI, OptionsFindStraightEdge Opts)
        {
            #region      ////////////           Khởi tạo các điểm và list điểm của ROI         /////////////////////////////////////
            // Các điểm base
            List<Point> ROIProcess = Get_ROIProcess(ROI, Opts);
            Point point1 = ROIProcess[0];
            Point point2 = ROIProcess[1];
            Point point3 = ROIProcess[2];
            Point point4 = ROIProcess[3];

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


            #region     ////////////////         Xác định cạnh             /////////////////////////////////////////////////////////
            // Từ list các điểm, tìm điểm kết thúc của mỗi cạnh (break point)
            List<int> list_index_BreakPoint = new List<int>() { 0 };         // List break point là list chứa index của các điểm đầu/cuối mỗi cạnh
            List<List<int>> list_index_PointEdges = new List<List<int>> { };     // Chứa index đầu, cuối của mỗi cạnh tìm được
            List<double> list_angle_vector = new List<double>();       // List chứa góc của mỗi 2 điểm
            for (int i = 0; i < list_Point_DST.Count - 1; i++)         // Mỗi n điểm liên tục có n-1 vector liên tục, dùng để tính góc
            {
                double angle_vector = Math.Atan2(list_Point_DST[i + 1].Y - list_Point_DST[i].Y, list_Point_DST[i + 1].X - list_Point_DST[i].X);
                angle_vector = angle_vector * 180 / Math.PI;   // Đổi từ Atan2 rad sang degree
                list_angle_vector.Add(angle_vector);
                //Console.WriteLine(angle_vector);
            }
            // Nếu list góc vector không rỗng (tức là bắt được từ 2 điểm trở lên) => tìm breakpoint
            if (list_angle_vector.Count != 0)
            {
                double base_edge_angle = list_angle_vector[0];         //Đặt góc của cạnh ban đầu là góc đầu tiên sau điểm break, sau đó trượt dần tới cuối để xác định breakpoint

                for (int i = 0; i < list_angle_vector.Count - 1; i++)
                {
                    double dangle1 = Math.Abs(list_angle_vector[i + 1] - list_angle_vector[i]);      //So sánh góc giữa 2 vector liên tục
                    double dangle2 = Math.Abs(list_angle_vector[i + 1] - base_edge_angle);      //So sánh góc giữa vector và vector lấy làm chuẩn của cạnh đang trượt
                    if (dangle1 > 180) { dangle1 = 360 - dangle1; }        // Góc lệch luôn nhỏ hơn 180 phòng trường hợp 2 vector có góc 179 và -179, lệch nhau 2 độ
                    if (dangle2 > 180) { dangle2 = 360 - dangle2; }

                    if (dangle1 > Opts.AllowedDeflectionAngle & dangle2 > Opts.AllowedDeflectionAngle)
                    {
                        list_index_BreakPoint.Add(i + 1);
                        base_edge_angle = list_angle_vector[i + 1];
                    }
                }
                list_index_BreakPoint.Add(list_Point_DST.Count - 1);
                // Đoạn này đã kết thúc list_index_BreakPoint, gồm điểm đầu, các điểm gãy và điểm kết thúc. Ex: (0, 4, 5, 10, 23)

                // Lọc cạnh theo số điểm, lấy về list index của cạnh thật. Ex: ((0, 4), (5, 10), (10, 23))
                for (int i = 0; i < list_index_BreakPoint.Count - 1; i++)
                {
                    if (list_index_BreakPoint[i + 1] - list_index_BreakPoint[i] >= 3)
                    {
                        list_index_PointEdges.Add(new List<int> { list_index_BreakPoint[i], list_index_BreakPoint[i + 1] });
                    }
                }
            }
            #endregion


            #region     //////////////////         Compare các cạnh, tính thông số cạnh trả về      //////////////////////////////////
            // Tìm hệ số ax + by + c = 0 của đường làm chuẩn 14:
            // Phần này để tính khoảng cách của các cạnh tới đường làm chuẩn
            double a = point4.Y - point1.Y;
            double b = point1.X - point4.X;
            double c = point1.Y * point4.X - point1.X * point4.Y;

            List<Point2f> ListPoint_TrueEdge = new List<Point2f>();

            if (list_index_PointEdges != null)
            {
                // Hàm tính thuộc tính của mỗi cạnh, trả về list<list<double>> gồm  thông tin góc lệch, khoảng cách, số điểm của cạnh đó
                List<List<double>> list_Info_Edges = CalculateEdges(list_Point_DST, list_index_PointEdges, a, b, c, angle14);

                if (Opts.LookFor == LookForEdge.FirstEdge) { ListPoint_TrueEdge = FindStraight_FirstEdge(list_Point_DST, list_Info_Edges, list_index_PointEdges); }
                else if (Opts.LookFor == LookForEdge.BestEdge) { ListPoint_TrueEdge = FindStraight_BestEdge(list_Point_DST, list_Info_Edges, list_index_PointEdges); }
            }


            double distance_EdgeCatch = 0;
            double angle_EdgeCatch = 0;
            List<Point> list_PointCatch = new List<Point>();

            if (ListPoint_TrueEdge.Count != 0)
            {
                Line2D line = Cv2.FitLine(ListPoint_TrueEdge, DistanceTypes.Huber, 0, 0.01, 0.01);
                //Console.WriteLine("{0}, {1}, {2}, {3}", line.Vx, line.Vy, line.X1, line.Y1);

                // Tạo 2 điểm để vẽ đường cắt
                Point Point1Catch = new Point(line.X1 - line.Vx * Source.Width, line.Y1 - line.Vy * Source.Width);
                Point Point2Catch = new Point(line.X1 + line.Vx * Source.Width, line.Y1 + line.Vy * Source.Width);

                list_PointCatch = new List<Point>() { Point1Catch, Point2Catch };

                // Tạo đường thẳng mx + ny + p = 0, tính khoảng cách
                double m = line.Vy;
                double n = -line.Vx;
                double p = -m * line.X1 - n * line.Y1;

                distance_EdgeCatch = Math.Abs(p) / Math.Sqrt(m * m + n * n);
                distance_EdgeCatch = Math.Round(distance_EdgeCatch, 3);

                // Tính góc của đường thẳng
                angle_EdgeCatch = -Math.Atan2(line.Vy, line.Vx) * 180 / Math.PI;
                angle_EdgeCatch = Math.Round(angle_EdgeCatch, 3);
            }
            #endregion


            ResultFindStraightEdge resultFindEdge = new ResultFindStraightEdge(listPoint14, listPoint23, list_Point_DST, list_PointCatch, distance_EdgeCatch, angle_EdgeCatch);

            return resultFindEdge;
        }


        /// <summary>
        /// Hàm sử dụng để thay đổi hướng xử lý của ROI
        /// </summary>
        public static List<Point> Get_ROIProcess(List<Point> ROI, Options Opts)
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
        /// Hàm tính thông số của mỗi cạnh tìm được
        /// Sử dụng cho Find Straight Edge
        /// </summary>
        public static List<List<double>> CalculateEdges(List<Point2f> ListPoint_DST, List<List<int>> List_index_PointEdges, double a, double b, double c, double Ori_angle)
        {
            // ax + by + c = 0 là cạnh làm gốc, góc Ori_angle là góc của cạnh làm gốc
            List<List<double>> List_Infor_Edges = new List<List<double>>();     // kết quả trả về, gồm góc lệch, khoảng cách, sô điểm của mỗi cạnh

            for (int i = 0; i < List_index_PointEdges.Count; i++)
            {
                // Phần tính góc lệch
                double y = ListPoint_DST[List_index_PointEdges[i][1]].Y - ListPoint_DST[List_index_PointEdges[i][0]].Y;     // y1 - y0
                double x = ListPoint_DST[List_index_PointEdges[i][1]].X - ListPoint_DST[List_index_PointEdges[i][0]].X;     // x1 - x0
                double angle_Edge = Math.Atan2(y, x) * 180 / Math.PI;

                double deviation_angle = Math.Abs(angle_Edge - Ori_angle);
                if (deviation_angle > 180) { deviation_angle = 360 - deviation_angle; }
                if (deviation_angle > 90) { deviation_angle = 180 - deviation_angle; }


                // Phần tính khoảng cách
                double d1 = Math.Abs(a * ListPoint_DST[List_index_PointEdges[i][0]].X + b * ListPoint_DST[List_index_PointEdges[i][0]].Y + c) / Math.Sqrt(a * a + b * b);
                double d2 = Math.Abs(a * ListPoint_DST[List_index_PointEdges[i][1]].X + b * ListPoint_DST[List_index_PointEdges[i][1]].Y + c) / Math.Sqrt(a * a + b * b);

                double average_distance = (d1 + d2) / 2;


                // Phần tính số điểm
                double numPoint = List_index_PointEdges[i][1] - List_index_PointEdges[i][0] + 1;

                //Console.WriteLine("gl, kc, sd: {0}, {1}, {2}", deviation_angle, average_distance, numPoint);
                List_Infor_Edges.Add(new List<double> { deviation_angle, average_distance, numPoint });
            }
            return List_Infor_Edges;
        }


        /// <summary>
        /// Hàm tìm ra list point của cạnh gần nhất
        /// Sử dụng cho Find Straight Edge
        /// </summary>
        public static List<Point2f> FindStraight_FirstEdge(List<Point2f> ListPoints_DST, List<List<double>> List_Info_Edges, List<List<int>> List_index_PointEdge)
        {
            List<Point2f> ListPoint_TrueEdge = new List<Point2f>();

            double min_distance = 999999;
            int true_edge = -1;
            for (int i = 0; i < List_index_PointEdge.Count; i++)
            {
                //Console.WriteLine("Hello1");
                if (List_Info_Edges[i][0] < 45 & List_Info_Edges[i][1] < min_distance)     // Điều kiện góc lệch cần nhỏ hơn 45 độ và khoảng cách càng nhỏ càng tốt
                {
                    true_edge = i;
                    min_distance = List_Info_Edges[i][1];
                    //Console.WriteLine(i);
                }
            }

            if (true_edge != -1)
            {
                for (int i = List_index_PointEdge[true_edge][0]; i <= List_index_PointEdge[true_edge][1]; i++)
                {
                    ListPoint_TrueEdge.Add(ListPoints_DST[i]);
                }
            }

            return ListPoint_TrueEdge;
        }


        /// <summary>
        /// Hàm tìm ra list point của cạnh nhiều điểm nhất
        /// Sử dụng cho Find Straight Edge
        /// </summary>
        public static List<Point2f> FindStraight_BestEdge(List<Point2f> ListPoints_DST, List<List<double>> List_Info_Edges, List<List<int>> List_index_PointEdge)
        {
            List<Point2f> ListPoint_TrueEdge = new List<Point2f>();

            int max_numPoint = 0;
            double min_angle = 90;
            int true_edge = -1;
            for (int i = 0; i < List_index_PointEdge.Count; i++)
            {
                //Console.WriteLine("Hello1");
                if (List_Info_Edges[i][2] > max_numPoint)     // Điều kiện số điểm max càng lớn càng tốt
                {
                    true_edge = i;
                    max_numPoint = (int)List_Info_Edges[i][2];
                    min_angle = List_Info_Edges[i][0];
                }
                else if (List_Info_Edges[i][2] == max_numPoint & List_Info_Edges[i][0] < min_angle) // Nếu số điểm bằng nhau, lấy góc tốt nhất
                {
                    true_edge = i;
                    min_angle = List_Info_Edges[i][0];
                    max_numPoint = (int)List_Info_Edges[i][2];
                }
            }

            if (true_edge != -1)
            {
                for (int i = List_index_PointEdge[true_edge][0]; i <= List_index_PointEdge[true_edge][1]; i++)
                {
                    ListPoint_TrueEdge.Add(ListPoints_DST[i]);
                }
            }

            return ListPoint_TrueEdge;
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
        public ResultFindStraightEdge(List<Point> listPoint_Grid_14, List<Point> listPoint_Grid_23, List<Point2f> listPoint_DST, List<Point> list_PointCatch, double distance_catch, double angle_catch)
        {
            ListPoint_Grid_14 = listPoint_Grid_14;
            ListPoint_Grid_23 = listPoint_Grid_23;
            ListPoint_DST = listPoint_DST;

            List_PointCatch = list_PointCatch;

            Distance_EdgeCatch = distance_catch;
            Angle_EdgeCatch = angle_catch;
        }
    }
}
