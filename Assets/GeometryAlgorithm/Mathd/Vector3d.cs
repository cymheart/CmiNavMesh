using System;

namespace Mathd
{
    public struct Vector3d
    {
        #region public members

        public double x;
        public double y;
        public double z;

        #endregion

        #region constructor

        public Vector3d(double p_x, double p_y)
        {
            x = p_x;
            y = p_y;
            z = 0;
        }
        public Vector3d(double p_x, double p_y, double p_z)
        {
            x = p_x;
            y = p_y;
            z = p_z;
        }

        #endregion

        #region public properties

        public double this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0:
                        return x;
                    case 1:
                        return y;
                    case 2:
                        return z;
                    default:
                        throw new IndexOutOfRangeException("Invalid Vector3d index!");
                }
            }
            set
            {
                switch (index)
                {
                    case 0:
                        x = value;
                        break;
                    case 1:
                        y = value;
                        break;
                    case 2:
                        z = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException("Invalid Vector3d index!");
                }
            }
        }

        public static Vector3d back
        {
            get
            {
                return new Vector3d(0, 0, -1);
            }
        }
        public static Vector3d down
        {
            get
            {
                return new Vector3d(0, -1, 0);
            }
        }
        public static Vector3d forward
        {
            get
            {
                return new Vector3d(0, 0, 1);
            }
        }
        public static Vector3d left
        {
            get
            {
                return new Vector3d(-1, 0, 0);
            }
        }
        public static Vector3d one
        {
            get
            {
                return new Vector3d(1, 1, 1);
            }
        }
        public static Vector3d right
        {
            get
            {
                return new Vector3d(1, 0, 0);
            }
        }
        public static Vector3d up
        {
            get
            {
                return new Vector3d(0, 1, 0);
            }
        }
        public static Vector3d zero
        {
            get
            {
                return new Vector3d(0, 0, 0);
            }
        }
        public double magnitude
        {
            get
            {
                return Math.Sqrt(sqrMagnitude);
            }
        }
        public Vector3d normalized
        {
            get
            {
                return Normalize(this);
            }
        }
        public double sqrMagnitude
        {
            get
            {
                return x * x + y * y + z * z;
            }
        }

        #endregion

        #region public functions

        /// <summary>
        /// 夹角大小
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public static float Angle(Vector3d from, Vector3d to)
        {
            double cos = Dot(from.normalized, to.normalized);
            if (cos < -1)
            {
                cos = -1;
            }
            if (cos > 1)
            {
                cos = 1;
            }
            return (float)(Math.Acos(cos) * (180/ Math.PI));
        }
        /// <summary>
        /// 夹角大小（弧度）
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public static float AngleBetween(Vector3d from, Vector3d to)
        {
            double cos = Dot(from.normalized, to.normalized);
            if (cos < -1)
            {
                cos = -1;
            }
            if (cos > 1)
            {
                cos = 1;
            }
            return (float)(Math.Acos(cos));
        }
        /// <summary>
        /// 距离限制
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="maxLength"></param>
        /// <returns></returns>
        public static Vector3d ClampMagnitude(Vector3d vector, double maxLength)
        {
            if (vector.sqrMagnitude > maxLength * maxLength)
            {
                return vector.normalized * maxLength;
            }
            else
            {
                return vector;
            }
        }
        /// <summary>
        /// 差乘
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public static Vector3d Cross(Vector3d lhs, Vector3d rhs)
        {
            double x = lhs.y * rhs.z - rhs.y * lhs.z;
            double y = lhs.z * rhs.x - rhs.z * lhs.x;
            double z = lhs.x * rhs.y - rhs.x * lhs.y;
            return new Vector3d(x, y, z);
        }
        /// <summary>
        /// 距离
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static double Distance(Vector3d a, Vector3d b)
        {
            return Math.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
        }
        /// <summary>
        /// 点乘
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public static double Dot(Vector3d lhs, Vector3d rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }
        /// <summary>
        /// 去除
        /// </summary>
        /// <param name="excludeThis"></param>
        /// <param name="fromThat"></param>
        /// <returns></returns>
        public static Vector3d Exclude(Vector3d excludeThis, Vector3d fromThat)
        {
            return fromThat - Project(fromThat, excludeThis);
        }
        /// <summary>
        /// 线性插值
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static Vector3d Lerp(Vector3d a, Vector3d b, double t)
        {
            if (t <= 0)
            {
                return a;
            }
            else if (t >= 1)
            {
                return b;
            }
            return a + (b - a) * t;
        }
        /// <summary>
        /// 线性插值(无限制)
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static Vector3d LerpUnclamped(Vector3d a, Vector3d b, double t)
        {
            return a + (b - a) * t;
        }
        /// <summary>
        /// 向量模长
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public static double Magnitude(Vector3d a)
        {
            return a.magnitude;
        }
        /// <summary>
        /// 最大值(X,Y,Z均取最大)
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public static Vector3d Max(Vector3d lhs, Vector3d rhs)
        {
            Vector3d temp = new Vector3d();
            temp.x = Math.Max(lhs.x, rhs.x);
            temp.y = Math.Max(lhs.y, rhs.y);
            temp.z = Math.Max(lhs.z, rhs.z);
            return temp;
        }
        /// <summary>
        /// 最小值(X,Y,Z均取最小)
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public static Vector3d Min(Vector3d lhs, Vector3d rhs)
        {
            Vector3d temp = new Vector3d();
            temp.x = Math.Min(lhs.x, rhs.x);
            temp.y = Math.Min(lhs.y, rhs.y);
            temp.z = Math.Min(lhs.z, rhs.z);
            return temp;
        }
        /// <summary>
        /// 向目标点移动
        /// </summary>
        /// <param name="current"></param>
        /// <param name="target"></param>
        /// <param name="maxDistanceDelta"></param>
        /// <returns></returns>
        public static Vector3d MoveTowards(Vector3d current, Vector3d target, double maxDistanceDelta)
        {
            Vector3d vector3 = target - current;
            double single = vector3.magnitude;
            if (single <= maxDistanceDelta || single == 0f)
            {
                return target;
            }
            return current + ((vector3 / single) * maxDistanceDelta);
        }
        /// <summary>
        /// 单位化
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public static Vector3d Normalize(Vector3d value)
        {
            if (value == zero)
            {
                return zero;
            }
            else
            {
                Vector3d tempDVec = new Vector3d();
                tempDVec.x = value.x / value.magnitude;
                tempDVec.y = value.y / value.magnitude;
                tempDVec.z = value.z / value.magnitude;
                return tempDVec;
            }
        }
        /// <summary>
        /// 正交法线
        /// </summary>
        /// <param name="normal"></param>
        /// <param name="tangent"></param>
        public static void OrthoNormalize(ref Vector3d normal, ref Vector3d tangent)
        {
            double mag = Magnitude(normal);
            if (mag > 0)
                normal /= mag;
            else
                normal = new Vector3d(1, 0, 0);

            double dot0 = Dot(normal, tangent);
            tangent -= dot0 * normal;
            mag = Magnitude(tangent);
            if (mag < 0)
                tangent = OrthoNormalVectorFast(normal);
            else
                tangent /= mag;
        }
        /// <summary>
        /// 正交法线
        /// </summary>
        /// <param name="normal"></param>
        /// <param name="tangent"></param>
        /// <param name="binormal"></param>
        public static void OrthoNormalize(ref Vector3d normal, ref Vector3d tangent, ref Vector3d binormal)
        {
            double mag = Magnitude(normal);
            if (mag > 0)
                normal /= mag;
            else
                normal = new Vector3d(1, 0, 0);

            double dot0 = Dot(normal, tangent);
            tangent -= dot0 * normal;
            mag = Magnitude(tangent);
            if (mag > 0)
                tangent /= mag;
            else
                tangent = OrthoNormalVectorFast(normal);

            double dot1 = Dot(tangent, binormal);
            dot0 = Dot(normal, binormal);
            binormal -= dot0 * normal + dot1 * tangent;
            mag = Magnitude(binormal);
            if (mag > 0)
                binormal /= mag;
            else
                binormal = Cross(normal, tangent);
        }
        /// <summary>
        /// 向量投影
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="onNormal"></param>
        /// <returns></returns>
        public static Vector3d Project(Vector3d vector, Vector3d onNormal)
        {
            if (vector == zero || onNormal == zero)
            {
                return zero;
            }
            return Dot(vector, onNormal) / (onNormal.magnitude * onNormal.magnitude) * onNormal;
        }
        /// <summary>
        /// 向量在平面上的投影
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="planeNormal"></param>
        /// <returns></returns>
        public static Vector3d ProjectOnPlane(Vector3d vector, Vector3d planeNormal)
        {
            return vector - Project(vector, planeNormal);
        }
        /// <summary>
        /// 反射
        /// </summary>
        /// <param name="inDirection"></param>
        /// <param name="inNormal"></param>
        /// <returns></returns>
        public static Vector3d Reflect(Vector3d inDirection, Vector3d inNormal)
        {
            return (-2f * Dot(inNormal, inDirection)) * inNormal + inDirection;
        }

        /// <summary>
        /// 向量缩放
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static Vector3d Scale(Vector3d a, Vector3d b)
        {
            Vector3d temp = new Vector3d();
            temp.x = a.x * b.x;
            temp.y = a.y * b.y;
            temp.z = a.z * b.z;
            return temp;
        }
 
        /// <summary>
        /// 模长平方
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public static double SqrMagnitude(Vector3d a)
        {
            return a.sqrMagnitude;
        }
        /// <summary>
        /// 单位化
        /// </summary>
        public void Normalize()
        {
            if (this != zero)
            {
                double length = magnitude;
                x /= length;
                y /= length;
                z /= length;
            }
        }
        /// <summary>
        /// 缩放
        /// </summary>
        /// <param name="scale"></param>
        public void Scale(Vector3d scale)
        {
            x *= scale.x;
            y *= scale.y;
            z *= scale.z;
        }
        /// <summary>
        /// 设置向量
        /// </summary>
        /// <param name="new_x"></param>
        /// <param name="new_y"></param>
        /// <param name="new_z"></param>
        public void Set(double new_x, double new_y, double new_z)
        {
            x = new_x;
            y = new_y;
            z = new_z;
        }
        public override string ToString()
        {
            return String.Format("({0}, {1}, {2})", x, y, z);
        }
        public override int GetHashCode()
        {
            return this.x.GetHashCode() ^ this.y.GetHashCode() << 2 ^ this.z.GetHashCode() >> 2;
        }
        public override bool Equals(object other)
        {
            return this == (Vector3d)other;
        }
        public string ToString(string format)
        {
            return String.Format("({0}, {1}, {2})", x.ToString(format), y.ToString(format), z.ToString(format));
        }

        #endregion

        #region private functions

        private static Vector3d OrthoNormalVectorFast(Vector3d normal)
        {
            double k1OverSqrt2 = Math.Sqrt(0.5);
            Vector3d res;
            if (Math.Abs(normal.z) > k1OverSqrt2)
            {
                double a = normal.y * normal.y + normal.z * normal.z;
                double k = 1 / Math.Sqrt(a);
                res.x = 0;
                res.y = -normal.z * k;
                res.z = normal.y * k;
            }
            else
            {
                double a = normal.x * normal.x + normal.y * normal.y;
                double k = 1 / Math.Sqrt(a);
                res.x = -normal.y * k;
                res.y = normal.x * k;
                res.z = 0;
            }
            return res;
        }
        private static double ClampedMove(double lhs, double rhs, double clampedDelta)
        {
            double delta = rhs - lhs;
            if (delta > 0.0F)
                return lhs + Math.Min(delta, clampedDelta);
            else
                return lhs - Math.Min(-delta, clampedDelta);
        }
       
        #endregion

        #region operator

        public static Vector3d operator +(Vector3d a, Vector3d b)
        {
            return new Vector3d(a.x + b.x, a.y + b.y, a.z + b.z);
        }
        public static Vector3d operator -(Vector3d a)
        {
            return new Vector3d(-a.x, -a.y, -a.z);
        }
        public static Vector3d operator -(Vector3d a, Vector3d b)
        {
            return new Vector3d(a.x - b.x, a.y - b.y, a.z - b.z);
        }
        public static Vector3d operator *(double d, Vector3d a)
        {
            return new Vector3d(a.x * d, a.y * d, a.z * d);
        }
        public static Vector3d operator *(Vector3d a, double d)
        {
            return new Vector3d(a.x * d, a.y * d, a.z * d);
        }
        public static Vector3d operator /(Vector3d a, double d)
        {
            return new Vector3d(a.x / d, a.y / d, a.z / d);
        }
        public static bool operator ==(Vector3d lhs, Vector3d rhs)
        {
            if (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        public static bool operator !=(Vector3d lhs, Vector3d rhs)
        {
            return !(lhs == rhs);
        }

        #endregion
    }
}
