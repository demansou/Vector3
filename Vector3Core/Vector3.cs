using System;
using System.Runtime.Serialization;

namespace Vector3Core
{
    public class Vector3 : ISerializable
    {

        #region Fields

        public static readonly Vector3 Origin = new Vector3(0, 0, 0);
        public static readonly Vector3 XAxis = new Vector3(1, 0, 0);
        public static readonly Vector3 YAxis = new Vector3(0, 1, 0);
        public static readonly Vector3 ZAxis = new Vector3(0, 0, 1);
        public static readonly Vector3 MinValue = new Vector3(double.MinValue, double.MinValue, double.MinValue);
        public static readonly Vector3 MaxValue = new Vector3(double.MaxValue, double.MaxValue, double.MaxValue);
        public static readonly Vector3 Epsilon = new Vector3(double.Epsilon, double.Epsilon, double.Epsilon);
        public static readonly Vector3 Zero = Origin;
        public static readonly Vector3 NaN = new Vector3(double.NaN, double.NaN, double.NaN);

        #endregion

        #region Properties

        public double X { get; }

        public double Y { get; }

        public double Z { get; }

        public double[] Array => new[] { X, Y, Z };

        #endregion

        #region CTOR

        public Vector3(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public Vector3(double[] arr)
        {
            if (arr.Length != 3)
                throw new InvalidOperationException();

            X = arr[0];
            Y = arr[1];
            Z = arr[2];
        }

        public Vector3(Vector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        #endregion

        #region Arithmetic

        public double SumComponents()
        {
            return X + Y + Z;
        }

        public Vector3 PowComponents(double power)
        {
            return new Vector3(
                Math.Pow(X, power),
                Math.Pow(Y, power),
                Math.Pow(Z, power));
        }

        public Vector3 SqrComponents()
        {
            return PowComponents(2);
        }

        public Vector3 SqrtComponents()
        {
            return new Vector3(
                Math.Sqrt(X),
                Math.Sqrt(Y),
                Math.Sqrt(Z));
        }

        public double SumComponentSqrs()
        {
            return SqrComponents().SumComponents();
        }

        public double Magnitude()
        {
            return Math.Sqrt(SumComponentSqrs());
        }

        public Vector3 CrossProduct(Vector3 v)
        {
            return new Vector3(
                Y * v.Z - Z * v.Y,
                Z * v.X - X * v.Z,
                X * v.Y - Y * v.X);
        }

        public double DotProduct(Vector3 v)
        {
            return X * v.X + Y * v.Y + Z * v.Z;
        }

        public bool IsUnitVector()
        {
            return Magnitude() == 1;
        }

        public bool IsUnitVector(double tolerance)
        {
            return AlmostEqualsWithAbsTolerance(Magnitude(), 1, tolerance);
        }

        public bool IsNaN()
        {
            return double.IsNaN(X) || double.IsNaN(Y) || double.IsNaN(Z);
        }

        public Vector3 Normalize()
        {
            Vector3 returnVal;
            var magnitude = Magnitude();

            if (magnitude == 0 || double.IsNaN(magnitude))
            {
                returnVal = null;
            }
            else if (double.IsInfinity(magnitude))
            {
                var x = TransformDouble(X);
                var y = TransformDouble(Y);
                var z = TransformDouble(Z);

                var result = new Vector3(x, y, z);

                returnVal = !result.IsNaN()
                    ? result
                    : null;
            }
            else
            {
                returnVal = NormalizeOrNaN();
            }

            return returnVal;
        }

        public Vector3 NormalizeOrDefault()
        {
            return Normalize() ?? Origin;
        }

        private Vector3 NormalizeOrNaN()
        {
            var inverse = 1 / Magnitude();

            return new Vector3(
                X * inverse,
                Y * inverse,
                Z * inverse);
        }

        public double Abs()
        {
            return Magnitude();
        }

        public double Angle(Vector3 other)
        {
            if (Equals(other))
                return 0;

            var dotProduct = NormalizeOrDefault().DotProduct(other.NormalizeOrDefault());
            var min = Math.Min(1.0f, dotProduct);

            return Math.Acos(min);
        }

        public bool IsBackFace(Vector3 lineOfSight)
        {
            return Normalize().DotProduct(lineOfSight.Normalize()) < 0;
        }

        public double Distance(Vector3 other)
        {
            return Math.Sqrt(
                (X - other.X) * (X - other.X)
                + (Y - other.Y) * (Y - other.Y)
                + (Z - other.Z) * (Z - other.Z));
        }

        public Vector3 Interpolate(Vector3 other, double control, bool allowExtrapolation)
        {
            if (!allowExtrapolation && (control > 1 || control < 0))
                return null;

            return new Vector3(
                X * (1 - control) + other.X * control,
                Y * (1 - control) + other.Y * control,
                Z * (1 - control) + other.Z * control);
        }

        public Vector3 Interpolate(Vector3 other, double control)
        {
            return Interpolate(other, control, false);
        }

        public Vector3 Max(Vector3 other)
        {
            return this > other
                ? this
                : other;
        }

        public Vector3 Min(Vector3 other)
        {
            return this <= other
                ? this
                : other;
        }

        public double MixedProduct(Vector3 other1, Vector3 other2)
        {
            return CrossProduct(other1).DotProduct(other2);
        }

        public bool IsPerpendicular(Vector3 other)
        {
            var v1 = NormalizeSpecialCasesOrOriginal(this);
            var v2 = NormalizeSpecialCasesOrOriginal(other);

            if (v1 == Zero || v2 == Zero)
                return false;

            return v1.DotProduct(v2).Equals(0);
        }

        public bool IsPerpendicular(Vector3 other, double tolerance)
        {
            var v1 = NormalizeSpecialCasesOrOriginal(this);
            var v2 = NormalizeSpecialCasesOrOriginal(other);

            if (v1 == Zero || v2 == Zero)
                return false;

            var dotProduct = v1.DotProduct(v2);

            return AlmostEqualsWithAbsTolerance(dotProduct, 0, tolerance);
        }

        public Vector3 Projection(Vector3 direction)
        {
            var newVectorCoords = direction * (DotProduct(direction) / Math.Pow(direction.Magnitude(), 2));

            return new Vector3(newVectorCoords);
        }

        public Vector3 Rejection(Vector3 direction)
        {
            return this - Projection(direction);
        }

        public Vector3 Reflection(Vector3 reflector)
        {
            if (Math.Abs(Math.Abs(Angle(reflector)) - Math.PI / 2) < double.Epsilon)
                return -this;

            var returnVal = new Vector3(2 * Projection(reflector) - this);

            return returnVal.Scale(Magnitude());
        }

        public Vector3 RotateX(double rad)
        {
            var y = Y * Math.Cos(rad) - Z * Math.Sin(rad);
            var z = Y * Math.Sin(rad) + Z * Math.Cos(rad);

            return new Vector3(X, y, z);
        }

        public Vector3 Pitch(double rad)
        {
            return RotateX(rad);
        }

        public Vector3 RotateY(double rad)
        {
            var x = Z * Math.Sin(rad) + X * Math.Cos(rad);
            var z = Z * Math.Cos(rad) - X * Math.Sin(rad);

            return new Vector3(x, Y, z);
        }

        public Vector3 Yaw(double rad)
        {
            return RotateY(rad);
        }

        public Vector3 RotateZ(double rad)
        {
            var x = X * Math.Cos(rad) - Y * Math.Sin(rad);
            var y = X * Math.Sin(rad) + Y * Math.Cos(rad);

            return new Vector3(x, y, Z);
        }

        public Vector3 Roll(double rad)
        {
            return RotateZ(rad);
        }

        public Vector3 RotateX(double yOffset, double zOffset, double rad)
        {
            var y = Y * Math.Cos(rad) - Z * Math.Sin(rad)
                    + (yOffset * (1 - Math.Cos(rad)) + zOffset * Math.Sin(rad));
            var z = Y * Math.Sin(rad) + Z * Math.Cos(rad)
                    + (zOffset * (1 - Math.Cos(rad)) - yOffset * Math.Sin(rad));

            return new Vector3(X, y, z);
        }

        public Vector3 RotateY(double xOffset, double zOffset, double rad)
        {
            var x = Z * Math.Sin(rad) + X * Math.Cos(rad)
                    + (xOffset * (1 - Math.Cos(rad)) - zOffset * Math.Sin(rad));
            var z = Z * Math.Cos(rad) - X * Math.Sin(rad)
                    + (zOffset * (1 - Math.Cos(rad)) - xOffset * Math.Sin(rad));

            return new Vector3(x, Y, z);
        }

        public Vector3 RotateZ(double xOffset, double yOffset, double rad)
        {
            var x = X * Math.Cos(rad) - Y * Math.Sin(rad)
                    + (xOffset * (1 - Math.Cos(rad)) + yOffset * Math.Sin(rad));
            var y = X * Math.Sin(rad) + Y * Math.Sin(rad)
                    + (yOffset * (1 - Math.Cos(rad)) - xOffset * Math.Sin(rad));

            return new Vector3(x, y, Z);
        }

        public Vector3 Round()
        {
            return new Vector3(
                Math.Round(X),
                Math.Round(Y),
                Math.Round(Z));
        }

        public Vector3 Round(MidpointRounding mode)
        {
            return new Vector3(
                Math.Round(X, mode),
                Math.Round(Y, mode),
                Math.Round(Z, mode));
        }

        public Vector3 Round(int digits)
        {
            return new Vector3(
                Math.Round(X, digits),
                Math.Round(Y, digits),
                Math.Round(Z, digits));
        }

        public Vector3 Round(int digits, MidpointRounding mode)
        {
            return new Vector3(
                Math.Round(X, digits, mode),
                Math.Round(Y, digits, mode),
                Math.Round(Z, digits, mode));
        }

        public Vector3 Scale(double magnitude)
        {
            if (magnitude < 0 || this == Zero)
                return null;

            return this * (magnitude / Magnitude());
        }

        #endregion

        #region Operator Overrides

        public static Vector3 operator +(Vector3 v1, Vector3 v2)
        {
            return new Vector3(
                v1.X + v2.X,
                v1.Y + v2.Y,
                v1.Z + v2.Z);
        }

        public static Vector3 operator +(Vector3 v)
        {
            return new Vector3(
                +v.X,
                +v.Y,
                +v.Z);
        }

        public static Vector3 operator -(Vector3 v1, Vector3 v2)
        {
            return new Vector3(
                v1.X - v2.X,
                v1.Y - v2.Y,
                v1.Z - v2.Z);
        }

        public static Vector3 operator -(Vector3 v)
        {
            return new Vector3(
                -v.X,
                -v.Y,
                -v.Z);
        }

        public static Vector3 operator *(Vector3 v, double d)
        {
            return new Vector3(
                v.X * d,
                v.Y * d,
                v.Z * d);
        }

        public static Vector3 operator *(double d, Vector3 v)
        {
            return v * d;
        }

        public static Vector3 operator /(Vector3 v, double d)
        {
            return new Vector3(
                v.X / d,
                v.Y / d,
                v.Z / d);
        }

        public static bool operator <(Vector3 v1, Vector3 v2)
        {
            return v1.SumComponentSqrs() < v2.SumComponentSqrs();
        }

        public static bool operator >(Vector3 v1, Vector3 v2)
        {
            return v1.SumComponentSqrs() > v2.SumComponentSqrs();
        }

        public static bool operator <=(Vector3 v1, Vector3 v2)
        {
            return v1.SumComponentSqrs() <= v2.SumComponentSqrs();
        }

        public static bool operator >=(Vector3 v1, Vector3 v2)
        {
            return v1.SumComponentSqrs() >= v2.SumComponentSqrs();
        }

        public static bool operator ==(Vector3 v1, Vector3 v2)
        {
            return v1.X == v2.X
                   && v1.Y == v2.Y
                   && v1.Z == v2.Z;
        }

        public static bool operator !=(Vector3 v1, Vector3 v2)
        {
            return !(v1 == v2);
        }

        #endregion

        #region Other

        public string ToString(string format, IFormatProvider formatProvider)
        {
            if (string.IsNullOrWhiteSpace(format))
                return $"{X}, {Y}, {Z}";

            var firstChar = format[0];
            string remainder = null;

            if (format.Length > 1)
                remainder = format.Substring(1);

            string returnVal;
            switch (firstChar)
            {
                case 'x':
                    returnVal = X.ToString(remainder, formatProvider);
                    break;
                case 'y':
                    returnVal = Y.ToString(remainder, formatProvider);
                    break;
                case 'z':
                    returnVal = Z.ToString(remainder, formatProvider);
                    break;
                default:
                    returnVal =
                        string.Format("{0}, {1}, {2}",
                            X.ToString(format, formatProvider),
                            Y.ToString(format, formatProvider),
                            Z.ToString(format, formatProvider));
                    break;
            }

            return returnVal;
        }

        #endregion

        #region Default Overrides

        public override string ToString()
        {
            return $"Vector3 ({X} {Y} {Z})";
        }

        public override bool Equals(object obj)
        {
            var returnVal = false;

            if (obj is Vector3 otherObj)
            {
                returnVal = X.Equals(otherObj.X)
                            && Y.Equals(otherObj.Y)
                            && Z.Equals(otherObj.Z);
            }

            return returnVal;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = X.GetHashCode();
                hashCode = (hashCode * 397) ^ Y.GetHashCode();
                hashCode = (hashCode * 397) ^ Z.GetHashCode();
                return hashCode;
            }
        }

        #endregion

        #region Static

        private static double TransformDouble(double num)
        {
            return num == 0
                ? 0
                : num == -0
                    ? 0
                    : double.IsPositiveInfinity(num)
                        ? 1
                        : double.IsNegativeInfinity(num)
                            ? -1
                            : double.NaN;
        }

        private static Vector3 NormalizeSpecialCasesOrOriginal(Vector3 v)
        {
            var x = TransformDouble(v.X);
            var y = TransformDouble(v.Y);
            var z = TransformDouble(v.Z);

            return new Vector3(x, y, z);
        }

        private static bool AlmostEqualsWithAbsTolerance(double a, double b, double maxAbsoluteError)
        {
            var diff = Math.Abs(a - b);

            if (a.Equals(b))
                return true;

            return diff <= maxAbsoluteError;
        }

        #endregion

        #region ISerializable

        public void GetObjectData(SerializationInfo info, StreamingContext context)
        {
            info.AddValue(nameof(X), X);
            info.AddValue(nameof(Y), Y);
            info.AddValue(nameof(Z), Z);
        }

        #endregion

    }
}
