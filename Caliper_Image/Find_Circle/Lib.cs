///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////         LIB của FindStraightEdge RANSAC           /////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////



using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Linq;
using System.Reflection;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;
using OpenCvSharp;
using static OpenCvSharp.Stitcher;

namespace FindCircle
{
    public static class Gadgets
    {
        public static void FindCircle(SourceImage Source, ROI_Circle ROI, OptionsFindCircle Opts)
        {

            #region      ////////////////        Tìm các điểm của ROI tròn            /////////////////////////////////////////////
            //Cv2.Circle(Source, ROI.CenterPoint, 1, Scalar.Yellow, 2);
            //Cv2.Circle(Source, ROI.CenterPoint, (int)ROI.radius, Scalar.Red, 2);
            //Cv2.Circle(Source, ROI.CenterPoint, (int)ROI.Radius, Scalar.Red, 2);

            List<Point> listPoint_Inside = new List<Point>();     // List point bên trong
            List<Point> listPoint_Outside = new List<Point>();    // List point bên ngoài
            for (int i = 0; i < Opts.NumerLine; i++)
            {
                Point point_in_listInside = RotatePoint(new Point(0, ROI.radius), new Point(0, 0), i * 360 / Opts.NumerLine, ROI.CenterPoint);
                Point point_in_listOutside = RotatePoint(new Point(0, ROI.Radius), new Point(0, 0), i * 360 / Opts.NumerLine, ROI.CenterPoint);

                listPoint_Inside.Add(point_in_listInside);
                listPoint_Outside.Add(point_in_listOutside);

                //Cv2.Circle(Source, point_in_listInside, 1, Scalar.Yellow, 2);
                //Cv2.Circle(Source, point_in_listOutside, 1, Scalar.Yellow, 2);
            }
            #endregion


            #region          ////////////////          Xử lý bắt điểm                     //////////////////////////////////////////////
            //List<Point2f> list_point_catch = new List<Point2f> { };
            Point2f[] list_point_catch = new Point2f[Opts.NumerLine];

            for (int i = 0; i < Opts.NumerLine; i++)
            {
                //Cv2.Line(Source, listPoint_Inside[i], listPoint_Outside[i], Scalar.Green, 1);

                ///////////////////////////             Phần lấy list pixel theo góc của từng đường              ///////////////
                double angle = 0;       // Góc của đường thẳng này, đặt do biến cục bộ được sử dụng cần gtri khởi tạo
                List<Point> range_pix = new List<Point>();

                if (Opts.Direction == DirectionFindCircle.InsidetoOutside)
                {
                    angle = Math.Atan2(listPoint_Outside[i].Y - listPoint_Inside[i].Y, listPoint_Outside[i].X - listPoint_Inside[i].X) * 180 / Math.PI;
                    //range_pix = GetRangePixel(Source, listPoint_Inside[i], listPoint_Outside[i], angle);
                }
                else if (Opts.Direction == DirectionFindCircle.OutsidetoInside)
                {
                    angle = Math.Atan2(-listPoint_Outside[i].Y + listPoint_Inside[i].Y, -listPoint_Outside[i].X + listPoint_Inside[i].X) * 180 / Math.PI;
                    //range_pix = GetRangePixel(Source, listPoint_Outside[i], listPoint_Inside[i], angle);
                }
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////



                //////////////////////////       Phần xử lý hình ảnh theo góc của từng đường          ///////////////////////////
                // Lấy matrix sobel tương ứng theo góc ROI
                var data = SobelMatrix.GetSobelMatrix(angle, Source);// lấy ra ảnh Sobel từ đối tượng có lớp SourceImage.

                Mat ImgSobel1 = data[0];//Sobel bậc1

                Mat ImgSobel2 = data[1];//Sobel bậc2



                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////



                //////////////////////////                              Phần bắt điểm                 ///////////////////////////
                // Lấy hướng xử lý cho các điểm ()
                // Hướng này để biết pixel bắt được nằm ở đâu so với đường làm gốc
                double direc_x, direc_y;
                if ((angle >= -90) & (angle <= 90)) { direc_x = 1; }
                else { direc_x = -1; }
                if ((angle >= 0) & (angle <= 180)) { direc_y = 1; }
                else { direc_y = -1; }


                /* List<double> line_derivative1 = GetLineInImgSobel(ImgSobel1, range_pix);    //List các đường đạo hàm bậc 1
                 List<double> line_derivative2 = GetLineInImgSobel(ImgSobel2, range_pix);    //List các đường đạo hàm bậc 2*/

                listPixel line_derivative1 = GetRangePixelSobel(ImgSobel1, listPoint_Outside[i], listPoint_Inside[i], angle);
                listPixel line_derivative2 = GetRangePixelSobel(ImgSobel2, listPoint_Outside[i], listPoint_Inside[i], angle);


                Point2f Point_DST = new Point2f();      // Lưu trữ điểm bắt được bằng kiểu float cho độ chính xác subpixel

                double x, y;
                double subpix = FindPeakPoint(line_derivative1.line_derivative, line_derivative2.line_derivative, Opts);
                if (subpix != -1)
                {

                    if ((angle >= -45 & angle <= 45) | (angle >= 135 & angle <= 225) | (angle >= -225 & angle <= -135))
                    {
                        x = line_derivative1.reange_pix.X + direc_x * subpix;                                                   // x = x0 + dx
                        y = line_derivative1.reange_pix.Y + direc_y * subpix * Math.Abs(Math.Tan(angle * Math.PI / 180));   // y = y0 + dx*sin/cos
                    }
                    else
                    {
                        y = line_derivative1.reange_pix.Y + direc_y * subpix;                                                   // y = y0 + dy
                        x = line_derivative1.reange_pix.X + direc_x * subpix / Math.Abs(Math.Tan(angle * Math.PI / 180));   // x = x0 + dy*cos/sin
                    }

                    Point_DST = new Point2f((float)x, (float)y);
                    //Cv2.Circle(Source, (Point)Point_DST, 1, Scalar.Blue, 2);
                }

                list_point_catch[i] = Point_DST;
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            }

            #endregion


            #region    /////////////           Tìm đường tròn bằng RANSAC           //////////////////////////////////////////////////////////
            int N_pick = 10;           // Số lần lặp
            double threshC = 3;        // Độ lệch dài cho phép
            int max_countC = 0;           // Số điểm
            List<Point2f> listPointFinal = new List<Point2f>();
            // Bán kính cuối cùng thu được

            //Fisher-Yates algorithm
            Random random = new Random();
            void Shuffle<Point2d>(Point2d[] list)
            {
                int n = Opts.NumerLine;
                for (int i = 0; i < (n - 1); i++)
                {
                    int r = i + random.Next(n - i);
                    Point2d p = list[r];
                    list[r] = list[i];
                    list[i] = p;
                }
            }


            for (int i = 0; i <= N_pick; i++)
            {
                Shuffle<Point2f>(list_point_catch);
                int countC = 0;
                List<Point2f> listPointInner = new List<Point2f>();

                Point2f point3c = list_point_catch[0];//chọn ngẫu nhiên điểm thứ nhất
                Point2f point4c = list_point_catch[1];//chọn ngẫu nhiên điểm thứ hai
                Point2f point5c = list_point_catch[2];//chọn ngẫu nhiên điểm thứ ba

                //RANSAC theo đường tròn

                double[,] A = new double[,]
                {
                    { 2*point3c.X, 2*point3c.Y, -1 },
                    { 2*point4c.X, 2*point4c.Y, -1 },
                    { 2*point5c.X, 2*point5c.Y, -1 },
                };
                double[] B = new double[]
                {
                    point3c.X*point3c.X + point3c.Y*point3c.Y ,
                    point4c.X*point4c.X + point4c.Y*point4c.Y ,
                    point5c.X*point5c.X + point5c.Y*point5c.Y ,
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
                //C = invA*B
                double[] C = new double[]         // C = [a b c]
                {
                    invA[0, 0] * B[0] + invA[0, 1] * B[1] + invA[0, 2] * B[2],
                    invA[1, 0] * B[0] + invA[1, 1] * B[1] + invA[1, 2] * B[2],
                    invA[2, 0] * B[0] + invA[2, 1] * B[1] + invA[2, 2] * B[2]
                };
                double radius = Math.Sqrt(C[0] * C[0] + C[1] * C[1] - C[2]);     // r = a^2 + b^2 - c    

                for (int j = 0; j < Opts.NumerLine; j++)
                {
                    double distanceC = Math.Sqrt((list_point_catch[j].X - C[0]) * (list_point_catch[j].X - C[0]) +
                                                 (list_point_catch[j].Y - C[1]) * (list_point_catch[j].Y - C[1]));
                    if (distanceC > (radius - threshC) && distanceC < (radius + threshC))
                    {
                        countC++;
                        listPointInner.Add(list_point_catch[j]);
                    }
                }

                if (countC == Opts.NumerLine)
                {
                    listPointFinal = listPointInner;
                    break;
                }

                else if (countC > max_countC)
                {
                    max_countC = countC;
                    listPointFinal = listPointInner;
                }
            }

            RotatedRect resultOfCircle = Cv2.FitEllipse(listPointFinal);

            //Console.WriteLine("Bat duoc so diem la {0}/{1}", max_countC, list_point_catch.Count);


            //Console.WriteLine("Tam duong tron ({0}, {1}) - Ban kinh {2}", Center_Point_Catch.X, Center_Point_Catch.Y, ((resultOfCircle.Size.Width + resultOfCircle.Size.Height) / 4));
            //Console.WriteLine(resultOfCircle.Center.X + "-" + resultOfCircle.Center.Y + "-" + ((resultOfCircle.Size.Height + resultOfCircle.Size.Width)/4));


            #endregion

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
        /// Hàm trả về (1) list các point (Đoạn thẳng)
        /// Đoạn thẳng này chứa các pixel của đường bắt điểm
        /// </summary>
        public static listPixel GetRangePixelSobel(Mat Source, Point PointStart, Point PointEnd, double angle) // Hàm trả về list giá trị Sobel của 1 range_Pix và tọa độ điểm đầu tiên của range_pix.
        {
            //Tạo một rangepix, là một list chứa các vị trí pixel
            listPixel listPixel = new listPixel();
            List<double> RangePixSobel = new List<double>();


            //Tìm phương trình đường thẳng ax + by + c = 0, tìm góc của chúng để lấy các pixel thuộc đoạn đó
            double a = PointEnd.Y - PointStart.Y;
            double b = -(PointEnd.X - PointStart.X);
            double c = -PointStart.X * PointEnd.Y + PointStart.Y * PointEnd.X;

            #region    //////////////////   Lấy các pixel thuộc đường thẳng      ///////////////////////////////////
            if ((angle >= -45 & angle <= 45) || (angle >= 135 & angle <= 225) || (angle >= -255 & angle <= -135))     /////////// Tính pixel nếu đường ngang
            {
                if (b < 0)          // (Nếu là đường ngang) hướng theo chiều tăng X
                {
                    for (int x = PointStart.X; x <= PointEnd.X; x++)
                    {
                        double y = -(a * x + c) / b;
                        if (y >= 0 & y < Source.Height & x >= 0 & x < Source.Width)
                        {
                            //RangePix.Add(new Point(x, y));
                            RangePixSobel.Add(Source.Get<double>((int)y, x));// lưu giá trị Sobel tại điểm chỉ định.
                            if (x == PointStart.X)
                            {
                                listPixel.reange_pix = new Point2d(x, y); //lưu điểm đầu của range_pix.
                            }

                            //Source.Get<double>(x, y)
                            //Cv2.Circle(Source, new Point(x, (int)y), 2, Scalar.Red, 1);
                        }
                    }
                }
                else                // (Nếu là đường ngang) hướng theo chiều giảm X
                {
                    for (int x = PointStart.X; x >= PointEnd.X; x--)
                    {
                        double y = -(a * x + c) / b;
                        if (y >= 0 & y < Source.Height & x >= 0 & x < Source.Width)
                        {
                            RangePixSobel.Add(Source.Get<double>((int)y, x));// lưu giá trị Sobel tại điểm chỉ định.
                            if (x == PointStart.X)
                            {
                                listPixel.reange_pix = new Point2d(x, y);//lưu điểm đầu của range_pix.
                            }
                            //Cv2.Circle(Source, new Point(x, (int)y), 2, Scalar.Red, 1);
                        }
                    }
                }
            }

            else        ////////////////// Tính pixel nếu đường dọc
            {
                if (a > 0)           // (Nếu là đường dọc) hướng theo chiều tăng Y
                {
                    for (int y = PointStart.Y; y <= PointEnd.Y; y++)
                    {
                        double x = -(b * y + c) / a;
                        if (y >= 0 & y < Source.Height & x >= 0 & x < Source.Width)
                        {
                            RangePixSobel.Add(Source.Get<double>(y, (int)x));// lưu giá trị Sobel tại điểm chỉ định.
                            if (y == PointStart.Y)
                            {
                                listPixel.reange_pix = new Point2d(x, y);//lưu điểm đầu của range_pix.
                            }
                            //Cv2.Circle(Source, new Point((int)x, y), 2, Scalar.Red, 1);
                        }
                    }
                }
                else                 // (Nếu là đường dọc) hướng theo chiều giảm Y
                {
                    for (int y = PointStart.Y; y >= PointEnd.Y; y--)
                    {
                        double x = -(b * y + c) / a;
                        if (y >= 0 & y < Source.Height & x >= 0 & x < Source.Width)
                        {
                            RangePixSobel.Add(Source.Get<double>(y, (int)x));// lưu giá trị Sobel tại điểm chỉ định.
                            if (y == PointStart.Y)
                            {
                                listPixel.reange_pix = new Point2d(x, y);//lưu điểm đầu của range_pix.
                            }
                            //Cv2.Circle(Source, new Point((int)x, y), 2, Scalar.Red, 1);
                        }
                    }
                }
            }
            #endregion
            listPixel.line_derivative = RangePixSobel;
            return listPixel;

        }


        /// <summary>
        /// Hàm trả về (1) line
        /// Line là các giá trị độ xám trên các đường bắt điểm
        /// </summary>

        /// <summary>
        /// Hàm tìm các điểm biên cạnh
        /// </summary>
        public static double FindPeakPoint(List<double> line_derivative1, List<double> line_derivative2, OptionsFindCircle Opts)
        {
            double subpix = -1;                 // Giá trị subpix luôn dương, nếu trả về vẫn là -1 thì ko có điểm nào được bắt
            List<double> edge_final = new List<double> { };    // Tạo list để tìm gtri subpix


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



            #region   //////////////////////////////      Xử lý subpix              ///////////////////////////////////
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
            #endregion


            return subpix;
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
    public class listPixel //đối tượng lưu tọa điểm đầu tiên của range_pix và trả về giá trị Sobel của range_pix.
    {
        public List<double> line_derivative;
        public Point2d reange_pix;
    }
    public class OptionsFindCircle : Options
    {
        // Phần cài đặt riêng của Find Straight Edge
        public int NumerLine = 12;         // Số lượng đường tạo ra để bắt điểm (số điểm muốn bắt)
        public DirectionFindCircle Direction = DirectionFindCircle.InsidetoOutside;

        public OptionsFindCircle(int gap, int thresh)
        {
            Gap = gap;
            Thresh = thresh;
        }

        public OptionsFindCircle(int gap, int thresh, int numLine)
        {
            Gap = gap;
            Thresh = thresh;

            NumerLine = numLine;
        }

        public OptionsFindCircle(int gap, int thresh, DirectionFindCircle direction)
        {
            Gap = gap;
            Thresh = thresh;

            Direction = direction;
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
    /// Xoay hướng xử lý của ROI
    /// </summary>
    public enum Orientation
    {
        Default,
        RotateROI_90,
        RotateROI_180,
        RotateROI_270
    }


    /// <summary>
    /// Xoay hướng xử lý của ROI
    /// </summary>
    public enum DirectionFindCircle
    {
        InsidetoOutside,
        OutsidetoInside
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    #endregion


    /// <summary>
    /// Hàm lấy ma trận kernel phục vụ Sobel ảnh theo hướng của ROI
    /// </summary>
    public static class SobelMatrix
    {
        static public int[,] Matrix_LeftToRight = new int[,]     //Ma trix trái -> phải
        {
            { -1, 0, 1 },
            { -2, 0, 2 },
            { -1, 0, 1 }
        };

        static public int[,] Matrix_RightToLeft = new int[,]     //Ma trix phải -> trái
        {
            { 1, 0, -1 },
            { 2, 0, -2 },
            { 1, 0, -1 }
        };

        static public int[,] Matrix_TopToBottom = new int[,]        //Matrix trên -> dưới
        {
            { -1, -2, -1 },
            {  0,  0,  0 },
            {  1,  2,  1 },
        };

        static public int[,] Matrix_BottomToTop = new int[,]        //Matrix trên -> dưới
        {
            {  1,  2,  1 },
            {  0,  0,  0 },
            { -1, -2, -1 },
        };

        static public int[,] MatrixCross_LeftTopToRightBot = new int[,]    //Ma trix hướng trái trên -> phải dưới
        {
            { -2, -1,  0 },
            { -1,  0,  1 },
            {  0,  1,  2 },
        };

        static public int[,] MatrixCross_RightTopToLeftBot = new int[,]    //Ma trix hướng phải trên -> trái dưới
        {
            {  0, -1, -2 },
            {  1,  0, -1 },
            {  2,  1,  0 },
        };

        static public int[,] MatrixCross_LeftBotToRightTop = new int[,]    //Ma trix hướng trái dưới -> phải trên
        {
            {  0,  1,  2 },
            { -1,  0,  1 },
            { -2, -1,  0 },
        };

        static public int[,] MatrixCross_RightBotToLeftTop = new int[,]    //Ma trix hướng phải dưới -> trái trên
        {
            {  2,  1,  0 },
            {  1,  0, -1 },
            {  0, -1, -2 },
        };

        public static Mat[] GetSobelMatrix(double angle, SourceImage img)// hàm trả về ảnh Sobel theo giá trị góc truyền vào.
        {
            do { angle = angle + 360; }
            while (angle < -180);

            do { angle = angle - 360; }
            while (angle > 180);

            Mat[] matrix = new Mat[2];
            if ((angle >= -22.5 & angle <= 22.5))
            {
                //matrix = Matrix_LeftToRight;
                matrix = img.imgSobel_LeftToRight;
            }
            else if ((angle >= 157.5 & angle <= 180) | (angle >= -180 & angle <= -157.5))
            {
                //matrix = Matrix_RightToLeft;
                matrix = img.imgSobel_RightToLeft;
            }
            else if (angle >= 67.5 & angle <= 112.5)
            {
                //matrix = Matrix_TopToBottom;
                matrix = img.imgSobel_TopToBottom;
            }
            else if (angle >= -112.5 & angle <= -67.5)
            {
                //matrix = Matrix_BottomToTop;
                matrix = img.imgSobel_BottomToTop;
            }
            else if (angle > 22.5 & angle < 67.5)
            {
                //matrix = MatrixCross_LeftTopToRightBot;
                matrix = img.imgSobel_LeftTopToRightBot;
            }
            else if (angle > -67.5 & angle < -22.5)
            {
                //matrix = MatrixCross_LeftBotToRightTop;
                matrix = img.imgSobel_LeftBotToRightTop;
            }
            else if (angle > 112.5 & angle < 157.5)
            {
                //matrix = MatrixCross_RightTopToLeftBot;
                matrix = img.imgSobel_RightTopToLeftBot;
            }
            else if (angle > -157.5 & angle < -112.5)
            {
                //matrix = MatrixCross_RightBotToLeftTop;
                matrix = img.imgSobel_RightBotToLeftTop;
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
        public ResultFindStraightEdge(List<Point> listPoint_Grid_14, List<Point> listPoint_Grid_23, List<Point2f> listPoint_DST)
        {
            ListPoint_Grid_14 = listPoint_Grid_14;
            ListPoint_Grid_23 = listPoint_Grid_23;
            ListPoint_DST = listPoint_DST;
        }
    }


    /// <summary>
    /// ROI kiểu đường tròn, gồm 1 điểm và bán kính lớn, bán kính bé
    /// </summary>
    public class ROI_Circle
    {
        public Point CenterPoint;
        public double radius;          // Bán kính nhỏ
        public double Radius;          // Bán kính lớn

        public ROI_Circle(Point centerPoint, double ra, double Ra)
        {
            CenterPoint = centerPoint;
            radius = ra;
            Radius = Ra;
        }
    }
    public class SourceImage // đối tượng đầu vào của chương trình.
    {
        String path;
        public Mat imgOriginal = new Mat();
        public Mat ImgGray = new Mat();
        public Mat ImgBilateral = new Mat();

        //các array lưu các giá trị Sobel1 và Sobel 2 của ảnh
        public Mat[] imgSobel_RightToLeft = new Mat[2];
        public Mat[] imgSobel_LeftToRight = new Mat[2];
        public Mat[] imgSobel_TopToBottom = new Mat[2];
        public Mat[] imgSobel_BottomToTop = new Mat[2];

        public Mat[] imgSobel_LeftTopToRightBot = new Mat[2];
        public Mat[] imgSobel_LeftBotToRightTop = new Mat[2];
        public Mat[] imgSobel_RightTopToLeftBot = new Mat[2];
        public Mat[] imgSobel_RightBotToLeftTop = new Mat[2];


        public SourceImage(String path)//hàm khởi tạo đối tượng
        {
            this.path = path;
            imgOriginal = new Mat(path);
            Cv2.CvtColor(imgOriginal, ImgGray, ColorConversionCodes.BGR2GRAY);
            Cv2.BilateralFilter(ImgGray, ImgBilateral, 3, 55, 55);
            imgSobel_RightToLeft[0] = new Mat();
            imgSobel_RightToLeft[1] = new Mat();

            imgSobel_LeftToRight[0] = new Mat();
            imgSobel_LeftToRight[1] = new Mat();

            imgSobel_TopToBottom[0] = new Mat();
            imgSobel_TopToBottom[1] = new Mat();

            imgSobel_BottomToTop[0] = new Mat();
            imgSobel_BottomToTop[1] = new Mat();

            imgSobel_LeftTopToRightBot[0] = new Mat();
            imgSobel_LeftTopToRightBot[1] = new Mat();

            imgSobel_LeftBotToRightTop[0] = new Mat();
            imgSobel_LeftBotToRightTop[1] = new Mat();

            imgSobel_RightTopToLeftBot[0] = new Mat();
            imgSobel_RightTopToLeftBot[1] = new Mat();

            imgSobel_RightBotToLeftTop[0] = new Mat();
            imgSobel_RightBotToLeftTop[1] = new Mat();



            //Right to Left
            Cv2.Filter2D(ImgBilateral, imgSobel_RightToLeft[0], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.Matrix_RightToLeft));
            Cv2.Filter2D(imgSobel_RightToLeft[0], imgSobel_RightToLeft[1], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.Matrix_RightToLeft));
            //Left to Right
            Cv2.Filter2D(ImgBilateral, imgSobel_LeftToRight[0], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.Matrix_LeftToRight));
            Cv2.Filter2D(imgSobel_LeftToRight[0], imgSobel_LeftToRight[1], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.Matrix_LeftToRight));
            //Top to Bottom
            Cv2.Filter2D(ImgBilateral, imgSobel_TopToBottom[0], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.Matrix_TopToBottom));
            Cv2.Filter2D(imgSobel_TopToBottom[0], imgSobel_TopToBottom[1], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.Matrix_TopToBottom));
            //Bottom to Top
            Cv2.Filter2D(ImgBilateral, imgSobel_BottomToTop[0], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.Matrix_BottomToTop));
            Cv2.Filter2D(imgSobel_BottomToTop[0], imgSobel_BottomToTop[1], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.Matrix_BottomToTop));
            //Left Top to Right Bottom
            Cv2.Filter2D(ImgBilateral, imgSobel_LeftTopToRightBot[0], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.MatrixCross_LeftBotToRightTop));
            Cv2.Filter2D(imgSobel_LeftTopToRightBot[0], imgSobel_LeftTopToRightBot[1], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.MatrixCross_LeftBotToRightTop));
            //Left Bottom to Right Top
            Cv2.Filter2D(ImgBilateral, imgSobel_LeftBotToRightTop[0], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.MatrixCross_LeftBotToRightTop));
            Cv2.Filter2D(imgSobel_LeftBotToRightTop[0], imgSobel_LeftBotToRightTop[1], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.MatrixCross_LeftBotToRightTop));
            //Right Top to Left Bottom
            Cv2.Filter2D(ImgBilateral, imgSobel_RightTopToLeftBot[0], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.MatrixCross_RightTopToLeftBot));
            Cv2.Filter2D(imgSobel_RightTopToLeftBot[0], imgSobel_RightTopToLeftBot[1], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.MatrixCross_RightTopToLeftBot));
            //Right Bottom to Left Top
            Cv2.Filter2D(ImgBilateral, imgSobel_RightBotToLeftTop[0], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.MatrixCross_RightBotToLeftTop));
            Cv2.Filter2D(imgSobel_RightBotToLeftTop[0], imgSobel_RightBotToLeftTop[1], MatType.CV_64FC1, new Mat(rows: 3, cols: 3, type: MatType.CV_32SC1, SobelMatrix.MatrixCross_RightBotToLeftTop));

        }

        public void DisposeSource()//hàm giải phóng bộ nhớ.
        {
            imgOriginal.Dispose();
            ImgGray.Dispose();
            ImgBilateral.Dispose();


            imgSobel_RightToLeft[0].Dispose();
            imgSobel_LeftToRight[0].Dispose();
            imgSobel_TopToBottom[0].Dispose();
            imgSobel_BottomToTop[0].Dispose();

            imgSobel_LeftTopToRightBot[0].Dispose();
            imgSobel_LeftBotToRightTop[0].Dispose();
            imgSobel_RightTopToLeftBot[0].Dispose();
            imgSobel_RightBotToLeftTop[0].Dispose();

            imgSobel_RightToLeft[1].Dispose();
            imgSobel_LeftToRight[1].Dispose();
            imgSobel_TopToBottom[1].Dispose();
            imgSobel_BottomToTop[1].Dispose();

            imgSobel_LeftTopToRightBot[1].Dispose();
            imgSobel_LeftBotToRightTop[1].Dispose();
            imgSobel_RightTopToLeftBot[1].Dispose();
            imgSobel_RightBotToLeftTop[1].Dispose();
        }
    }
}
