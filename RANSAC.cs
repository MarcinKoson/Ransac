//Made by Marcin Kosoń

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Globalization;
using System.IO;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace ransacDLL
{
    [ComVisible(true)]
    [ClassInterface(ClassInterfaceType.AutoDual)]
    [Guid("3d788165-8436-4bb9-882b-b03c859f9ee3")]

    public class RANSAC
    {
        private class Point
        {
            public Point()
            {
                Cechy = new double[128];
            }
            public int ID;
            public double X;
            public double Y;
            public double ParamA;
            public double ParamB;
            public double ParamC;
            public double[] Cechy;

        }

        public RANSAC()
        {
            KeyPoints = new List<Point>();
            KeyPoints2 = new List<Point>();

            Nearest1 = new List<Tuple<int, int>>();
            Nearest2 = new List<Tuple<int, int>>();

            KeyPointPairs = new List<Tuple<int, int>>();

            PairKeyPoints2 = new List<Point>();
            PairKeyPoints = new List<Point>();

            InconsistentPairs = new List<Tuple<int, int>>();
            ConsistentPairs = new List<Tuple<int, int>>();

            RansacPairs = new List<Tuple<int, int>>();

            random = new Random();
        }
        private Random random;
        //Data loading
        private List<Point> KeyPoints { get; set; }
        private List<Point> KeyPoints2 { get; set; }
       
        private void LoadFile(string fileName)
        {
            Load(fileName, KeyPoints);
        }
        private void LoadFile2(string fileName)
        {
            Load(fileName, KeyPoints2);
        }
        private void Load(string fileName, List<Point> lista)
        {
            string[] linesLoad = File.ReadAllLines(fileName);

            List<string> lines = new List<string>(linesLoad);

            lines.RemoveAt(0);
            lines.RemoveAt(0);

            foreach (string line in lines)
            {
                string[] splitLine = line.Split(' ');
                Point newPointLoad = new Point
                {
                    ID = lista.Count,
                    X = double.Parse(splitLine[0], CultureInfo.InvariantCulture),
                    Y = double.Parse(splitLine[1], CultureInfo.InvariantCulture),
                    ParamA = double.Parse(splitLine[2], CultureInfo.InvariantCulture),
                    ParamB = double.Parse(splitLine[3], CultureInfo.InvariantCulture),
                    ParamC = double.Parse(splitLine[4], CultureInfo.InvariantCulture)
                };

                for (int i = 0; i < 128; i++)
                {
                    newPointLoad.Cechy[i] = double.Parse(splitLine[i + 5]);
                }
                lista.Add(newPointLoad);
            }
        }
        
        //Pairs determination
        private List<Tuple<int, int>> KeyPointPairs { get; set; }

        private void GetPairsOfKeyPoints()
        {
            for (int i = 0; i < KeyPoints.Count; i++)
            {
                int nearest = FindNearestNeighborsFor1(i);

                if (FindNearestNeighborsFor2(nearest) == i)
                {
                    //to para
                    KeyPointPairs.Add(new Tuple<int, int>(i, nearest));
                }

            }
        }

        private static readonly Func<Point, Point, double> Similarity1 =
          (punkt1, punkt2) =>
          {
              double value = 0;
              for (int i = 0; i < 128; i++)
              {
                  value += Math.Abs(punkt1.Cechy[i] - punkt2.Cechy[i]);
              }
              return value;
          };
        private Func<Point, Point, double> Similarity = Similarity1;   
        private int FindNearestNeighborsFor1(int id)
        {
            return FindNearestNeighbors(id, true);
        }
        private int FindNearestNeighborsFor2(int id)
        {
            return FindNearestNeighbors(id, false);
        }
        private  List<Tuple<int, int>> Nearest1 { get; set; }
        private  List<Tuple<int, int>> Nearest2 { get; set; }
        private int FindNearestNeighbors(int id, bool for1)
        {
            try
            {
                if (for1)
                {
                    return Nearest1.Find(x => x.Item1 == id).Item2;
                }
                else
                {
                    return Nearest2.Find(x => x.Item1 == id).Item2;
                }
            }
            catch (NullReferenceException)
            {
                return CalculateNearestNeighbors(id, for1);
            }

        }
        private int CalculateNearestNeighbors(int id, bool for1)
        {
            double value;
            int nearest;
            if (for1)
            {
                value = double.MaxValue;
                nearest = 0;

                for (int i = 0; i < KeyPoints2.Count; i++)
                {
                    double v = Similarity(KeyPoints[id], KeyPoints2[i]);

                    if (v < value)
                    {
                        value = v;
                        nearest = i;
                    }
                }
                Nearest1.Add(new Tuple<int, int>(id, nearest));
            }
            else
            {
                value = double.MaxValue;
                nearest = 0;

                for (int i = 0; i < KeyPoints.Count; i++)
                {
                    double v = Similarity(KeyPoints2[id], KeyPoints[i]);

                    if (v < value)
                    {
                        value = v;
                        nearest = i;
                    }
                }
                Nearest2.Add(new Tuple<int, int>(id, nearest));
            }
            return nearest;
        }
        //SPÓJNOŚĆ PAR=============================================
        private List<Tuple<int, int>> ConsistentPairs { get; set; }
        private List<Tuple<int, int>> InconsistentPairs { get; set; }

        private void KeyPointsPairsCohesion(int wielkoscSasiedztwa, int spojnoscOd)
        {
            foreach (var x in KeyPointPairs)
            {
                PairKeyPoints.Add(KeyPoints[x.Item1]);
                PairKeyPoints2.Add(KeyPoints2[x.Item2]);
            }

            foreach (var x in KeyPointPairs)
            {
                var Neiberhood1 = GetNearestPoints(x.Item1, wielkoscSasiedztwa, PairKeyPoints);
                var Neiberhood2 = GetNearestPoints(x.Item2, wielkoscSasiedztwa, PairKeyPoints2);
                int cohesion = 0;

                foreach (var n1id in Neiberhood1)
                    if (Neiberhood2.Contains(KeyPointPairs.Find(tup => tup.Item1 == n1id).Item2))
                    {
                        cohesion++;
                    }

                if (cohesion >= spojnoscOd)
                {
                    ConsistentPairs.Add(x);
                }
                else
                    InconsistentPairs.Add(x);
            }
        }

        private List<Point> PairKeyPoints { get; set; }
        private List<Point> PairKeyPoints2 { get; set; }
        private double EuclidDistance(Point a, Point b)
        {
            return Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
        }
        private List<int> GetNearestPoints(int PointID, int howMany, List<Point> poinstList)
        {
            Point orginPoint = poinstList.Find(x => x.ID == PointID);

            //TO DO znajduje siebie
            List<Tuple<int, double>> distances = new List<Tuple<int, double>>();

            foreach (Point p in poinstList)
            {
                distances.Add(new Tuple<int, double>(p.ID, EuclidDistance(orginPoint, p)));
            }

            //sort

            distances.Sort((x, y) => {
                if (x == null & y == null) return 0;
                if (x == null) return 1;
                if (y == null) return -1;
                if (x.Item2 == y.Item2) return 0;
                return (x.Item2 > y.Item2) ? 1 : -1;

            });
            List<int> returnList = new List<int>();
            for (int i = 1; i < howMany + 1; i++)
            {
                returnList.Add(distances[i].Item1);
            }
            return returnList;
        }

        //RANSAC
        private List<Tuple<int, int>> RansacPairs { get; set; }

        private void StartRansac(int iterationNumber, double maxError, int pairsCount = 3)
        {
            //Heurystyka modyfikacji rozkładu-> przygotowanie
            ConsistentPairsScore = new int[ConsistentPairs.Count];
            ConsistentPairsScoreSumAll = 0;
            for (int i = 0; i < ConsistentPairsScore.Length; i++)
            {
                ConsistentPairsScore[i] = 1;
                ConsistentPairsScoreSumAll++;
            }

            //--------------metoda
            Matrix<double> bestModel = null;
            int bestScore = 0;

            for (int i = 0; i < iterationNumber; i++)
            {
                Matrix<double> model;
                //TO DO heurystyka
                //Tuple<int, int>[] s = ChoseRandomWithDistanceHeuristic(pairsCount);
                Tuple<int, int>[] s = ChoseRandom(pairsCount);
                model = CalculateModel(s);

                int score = 0;

                foreach (var para in ConsistentPairs)
                {
                    double error = ModelError(model, para);
                    if (error < maxError)
                        score++;
                }

                if (score >= bestScore)
                {
                    bestScore = score;
                    bestModel = model;

                    foreach (var para in s)
                        ConsistentPairsScore[ConsistentPairs.FindIndex(x => x.Item1 == para.Item1)]++;
                }


            }

            //RERURN MODEL
            bestModelEND = bestModel;

            //PARY KONIEC

            foreach (var para in ConsistentPairs)
            {
                double error = ModelError(bestModelEND, para);
                if (error < maxError)
                    RansacPairs.Add(para);
            }

        }
        private double ModelError(Matrix<double> model, Tuple<int, int> pair)
        {
            Point a, b;
            a = PairKeyPoints.Find(x => x.ID == pair.Item1);
            b = PairKeyPoints2.Find(x => x.ID == pair.Item2);

            Matrix<double> matrix2 = DenseMatrix.OfArray(new double[,] {
                            {a.X},{a.Y},{1}
                           });

            matrix2 = model * matrix2;

            return EuclidDistance(new Point() { X = matrix2[0, 0], Y = matrix2[1, 0] }, b);

        }
        private Matrix<double> CalculateModel(Tuple<int, int>[] punkty)
        {
            Point a1, b1, a2, b2, a3, b3, a4, b4;

            a1 = PairKeyPoints.Find(x => x.ID == punkty[0].Item1);
            b1 = PairKeyPoints2.Find(x => x.ID == punkty[0].Item2);

            a2 = PairKeyPoints.Find(x => x.ID == punkty[1].Item1);
            b2 = PairKeyPoints2.Find(x => x.ID == punkty[1].Item2);

            a3 = PairKeyPoints.Find(x => x.ID == punkty[2].Item1);
            b3 = PairKeyPoints2.Find(x => x.ID == punkty[2].Item2);



            if (punkty.Length == 3)
            {
                Matrix<double> matrix1 = DenseMatrix.OfArray(new double[,] {
                            {a1.X,a1.Y,1 ,0,0,0},
                            {a2.X,a2.Y,1 ,0,0,0},
                            {a3.X,a3.Y,1 ,0,0,0},
                            {0,0,0, a1.X,a1.Y,1},
                            {0,0,0, a2.X,a2.Y,1},
                            {0,0,0, a3.X,a3.Y,1},

                });
                Matrix<double> matrix2 = DenseMatrix.OfArray(new double[,]{
                    {b1.X },{b2.X },{b3.X },{b1.Y },{b2.Y },{b3.Y }

                });

                matrix1 = matrix1.Inverse();
                //matrix1.Inver

                matrix2 = matrix1 * matrix2;


                Matrix<double> retMatrix = DenseMatrix.OfArray(new double[,] {
                    {matrix2[0,0],matrix2[1,0],matrix2[2,0] },
                    {matrix2[3,0],matrix2[4,0],matrix2[5,0] },
                    {0,0,1 }

                });

                return retMatrix;
            }
            if (punkty.Length == 4)
            {
                a4 = PairKeyPoints.Find(x => x.ID == punkty[3].Item1);
                b4 = PairKeyPoints2.Find(x => x.ID == punkty[3].Item2);
                //TO DO test perspektywa

                Matrix<double> matrix1 = DenseMatrix.OfArray(new double[,] {
                            {a1.X,a1.Y,1 ,0,0,0, -b1.X*a1.X, -b1.X*a1.Y},
                            {a2.X,a2.Y,1 ,0,0,0, -b2.X*a2.X, -b2.X*a2.Y},
                            {a3.X,a3.Y,1 ,0,0,0, -b3.X*a3.X, -b3.X*a3.Y},
                            {a4.X,a4.Y,1 ,0,0,0, -b4.X*a4.X, -b4.X*a4.Y},
                            {0,0,0, a1.X,a1.Y,1, -b1.Y*a1.X, -b1.Y*a1.Y},
                            {0,0,0, a2.X,a2.Y,1, -b2.Y*a2.X, -b2.Y*a2.Y},
                            {0,0,0, a3.X,a3.Y,1, -b3.Y*a3.X, -b3.Y*a3.Y},
                            {0,0,0, a4.X,a4.Y,1, -b4.Y*a4.X, -b4.Y*a4.Y},

                });
                Matrix<double> matrix2 = DenseMatrix.OfArray(new double[,]{
                    {b1.X },{b2.X },{b3.X },{b4.X },{b1.Y },{b2.Y },{b3.Y },{b4.Y}

                });

                matrix1 = matrix1.Inverse();
                //matrix1.Inver

                matrix2 = matrix1 * matrix2;


                Matrix<double> retMatrix = DenseMatrix.OfArray(new double[,] {
                    {matrix2[0,0],matrix2[1,0],matrix2[2,0] },
                    {matrix2[3,0],matrix2[4,0],matrix2[5,0] },
                    {matrix2[6,0],matrix2[7,0] ,1 }

                });

                return retMatrix;




            }
            else
            {
                throw new Exception("CalculateModel : Zła tabela");
            }

        }
        private Tuple<int, int>[] ChoseRandom(int howMany)
        {
            Random random = new Random();

            Tuple<int, int>[] returnTable = new Tuple<int, int>[howMany];

            for (int i = 0; i < howMany; i++)
            {
                returnTable[i] = ConsistentPairs[random.Next(ConsistentPairs.Count)];
            }
            return returnTable;

        }
        private Matrix<double> bestModelEND;
        //heuristics
        private Tuple<int, int>[] ChoseRandomWithDistanceHeuristic(int howMany, double r = 10, double R = 300)
        {
            Random random = new Random();

            Tuple<int, int>[] returnTable = new Tuple<int, int>[howMany];

            bool isBad = true;
            do
            {
                returnTable = new Tuple<int, int>[howMany];
                for (int i = 0; i < howMany; i++)
                {
                    returnTable[i] = ConsistentPairs[random.Next(ConsistentPairs.Count)];

                }
                isBad = false;

                for (int i = 0; i < howMany; i++)
                    for (int j = 0; j < howMany; j++)
                    {

                        if (i != j)
                        {
                            double distance = EuclidDistance(KeyPoints[returnTable[i].Item1], KeyPoints[returnTable[j].Item1]);
                            double distance2 = EuclidDistance(KeyPoints2[returnTable[i].Item2], KeyPoints2[returnTable[j].Item2]);
                            if (distance < r || distance2 < r || distance > R || distance2 > R)
                            {
                                isBad = true;
                                break;
                            }
                        }
                    }
            }
            while (isBad);

            return returnTable;
        }
        private  int ConsistentPairsScoreSumAll { set; get; }
        private int[] ConsistentPairsScore { set; get; }
        private  Tuple<int, int>[] ChoseRandomWithDistanceHeuristicAndModificationDistribution(int howMany, double r = 10, double R = 300)
        {

            Tuple<int, int>[] returnTable = new Tuple<int, int>[howMany];

            bool isBad = true;
            do
            {
                returnTable = new Tuple<int, int>[howMany];
                for (int i = 0; i < howMany; i++)
                {
                    returnTable[i] = ConsistentPairs[GetRandodmPairWithModificationDistrubution()];
                }
                isBad = false;

                for (int i = 0; i < howMany; i++)
                    for (int j = 0; j < howMany; j++)
                    {

                        if (i != j)
                        {
                            double distance = EuclidDistance(KeyPoints[returnTable[i].Item1], KeyPoints[returnTable[j].Item1]);
                            double distance2 = EuclidDistance(KeyPoints2[returnTable[i].Item2], KeyPoints2[returnTable[j].Item2]);
                            if (distance < r || distance2 < r || distance > R || distance2 > R)
                            {
                                isBad = true;
                                break;
                            }
                        }
                    }
            }
            while (isBad);

            return returnTable;
        }
        private int GetRandodmPairWithModificationDistrubution()
        {
            int number = random.Next(ConsistentPairsScoreSumAll);
            int id = ConsistentPairsScore.Length;

            for (int i = 0; i < ConsistentPairsScore.Length; i++)
            {
                number = number - ConsistentPairsScore[i];
                if (number < 0)
                {
                    id = i;
                    break;
                }
            }


            return id;
        }
        
        //
        public string Start(string file1, string file2,int sizeN,int cohesionN, int iterations, int maxError, int afiOrPersp)
        {
            try{
                LoadFile(file1);              
            }
            catch{ return "200"; }
            try{
                LoadFile2(file2);
            }
            catch{ return "201"; }
            try{
                GetPairsOfKeyPoints();
            }
            catch{ return "202"; }
            try{
                KeyPointsPairsCohesion(sizeN, cohesionN);
            }
            catch { return "203-" + KeyPointPairs.Count; }
            try {
                StartRansac(iterations, maxError, afiOrPersp);  //afi or perps -- AFI=3, PERS=4
            }
            catch {
                return "204-" + KeyPointPairs.Count + "-" + ConsistentPairs.Count ;
                }

            string returnString = "100-" + KeyPointPairs.Count + "-" + ConsistentPairs.Count +"-"+RansacPairs.Count +"/";

            foreach (var x in RansacPairs)
            {
                Point a, b;
                a = PairKeyPoints.Find(fi => fi.ID == x.Item1);
                b = PairKeyPoints2.Find(fi => fi.ID == x.Item2);
                //add to str
                returnString += a.X;
                returnString += ";";
                returnString += a.Y;
                returnString += ";";
                returnString += b.X;
                returnString += ";";
                returnString += b.Y;
                returnString += "/";
            }

            return returnString;
        }
    }
}
