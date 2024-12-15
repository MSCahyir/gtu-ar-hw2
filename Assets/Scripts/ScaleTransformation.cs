using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Vector3 = UnityEngine.Vector3;
using Matrix4x4 = UnityEngine.Matrix4x4;

public class ScaleTransformation : MonoBehaviour
{
    private List<Vector3> pointsSet1;
    private List<Vector3> pointsSet2;
    public GameObject pointPrefab;
    public GameObject linePrefab;
    public Material materialSet1;

    public void Driver()
    {
        List<Vector3> pointsSet3 = AlignAndScalePoints(pointsSet1, pointsSet2);
        VisualizePoints(pointsSet3, materialSet1);
        DrawMovementLines(pointsSet2, pointsSet3);
    }

    public void SetPoints(List<Vector3> newPointsSet1, List<Vector3> newPointsSet2)
    {
        pointsSet1 = newPointsSet1;
        pointsSet2 = newPointsSet2;
    }

    private void VisualizePoints(List<Vector3> points, Material material)
    {
        foreach (Vector3 point in points)
        {
            // Instantiate the prefab at each point's position
            GameObject pointObject = Instantiate(pointPrefab, point, Quaternion.identity);

            // Assign the material to the point
            Renderer pointRenderer = pointObject.GetComponent<Renderer>();
            if (pointRenderer != null)
            {
                pointRenderer.material = material;
            }
        }
    }

    private void DrawMovementLines(List<Vector3> originalPoints, List<Vector3> transformedPoints)
    {
        int minCount = Mathf.Min(originalPoints.Count, transformedPoints.Count);
        for (int i = 0; i < minCount; i++)
        {
            Vector3 start = originalPoints[i];
            Vector3 end = transformedPoints[i];

            // Instantiate the line renderer prefab
            GameObject lineObj = Instantiate(linePrefab, Vector3.zero, Quaternion.identity);
            LineRenderer lineRenderer = lineObj.GetComponent<LineRenderer>();

            // Set line positions
            lineRenderer.SetPosition(0, start);
            lineRenderer.SetPosition(1, end);
        }
    }

    private List<Vector3> AlignAndScalePoints(List<Vector3> sourcePoints, List<Vector3> targetPoints)
    {
        int minCount = Mathf.Min(sourcePoints.Count, targetPoints.Count);

        if (minCount < 3)
        {
            Debug.LogError("Point sets must have at least 3 points.");
            return null;
        }

        // Compute centroids
        Vector3 centroidSource = ComputeCentroid(sourcePoints.GetRange(0, minCount));
        Vector3 centroidTarget = ComputeCentroid(targetPoints.GetRange(0, minCount));

        // Center the points around the centroids
        List<Vector3> centeredSource = CenterPoints(sourcePoints.GetRange(0, minCount), centroidSource);
        List<Vector3> centeredTarget = CenterPoints(targetPoints.GetRange(0, minCount), centroidTarget);

        // Compute the rotation matrix using Kabsch algorithm
        Quaternion rotation = ComputeKabschRotation(centeredSource, centeredTarget);

        // Compute scale factor
        float scaleFactor = ComputeScaleFactor(sourcePoints, targetPoints);

        // Apply the rotation, scaling, and translation
        List<Vector3> transformedPoints = new List<Vector3>();
        for (int i = 0; i < minCount; i++)
        {
            Vector3 point = sourcePoints[i];
            Vector3 rotatedPoint = rotation * (point - centroidSource);
            Vector3 scaledAndTranslatedPoint = (rotatedPoint * scaleFactor) + centroidTarget;
            transformedPoints.Add(scaledAndTranslatedPoint);
        }

        return transformedPoints;
    }

    private Vector3 ComputeCentroid(List<Vector3> points)
    {
        Vector3 sum = Vector3.zero;
        foreach (Vector3 point in points)
        {
            sum += point;
        }
        return sum / points.Count;
    }

    private float ComputeScaleFactor(List<Vector3> sourcePoints, List<Vector3> targetPoints)
    {
        int minCount = Mathf.Min(sourcePoints.Count, targetPoints.Count);
        float totalScaleFactor = 0f;
        int validCount = 0; // Count of valid divisions

        for (int i = 0; i < minCount; i++)
        {
            float sourceMagnitude = sourcePoints[i].magnitude;

            // Check if sourceMagnitude is very small (close to zero) to avoid division by zero
            if (Mathf.Approximately(sourceMagnitude, 0f))
            {
                continue;
            }

            float targetMagnitude = targetPoints[i].magnitude;
            totalScaleFactor += targetMagnitude / sourceMagnitude;
            validCount++;
        }

        // Avoid division by zero if validCount is zero
        return validCount > 0 ? totalScaleFactor / validCount : 1f; // Default scale factor of 1 if no valid divisions
    }

    private List<Vector3> CenterPoints(List<Vector3> points, Vector3 centroid)
    {
        return points.Select(point => point - centroid).ToList();
    }

    private Quaternion ComputeKabschRotation(List<Vector3> P, List<Vector3> Q)
    {
        // Compute covariance matrix
        float[,] C = new float[3, 3];
        for (int i = 0; i < P.Count; i++)
        {
            C[0, 0] += P[i].x * Q[i].x;
            C[0, 1] += P[i].x * Q[i].y;
            C[0, 2] += P[i].x * Q[i].z;
            C[1, 0] += P[i].y * Q[i].x;
            C[1, 1] += P[i].y * Q[i].y;
            C[1, 2] += P[i].y * Q[i].z;
            C[2, 0] += P[i].z * Q[i].x;
            C[2, 1] += P[i].z * Q[i].y;
            C[2, 2] += P[i].z * Q[i].z;
        }

        // Compute the Singular Value Decomposition of C
        float[,] U, S, V;
        SVD(C, out U, out S, out V);

        // Compute rotation matrix
        float[,] R = MultiplyMatrices(U, TransposeMatrix(V));

        // Convert rotation matrix to Matrix4x4
        Matrix4x4 RMatrix = new Matrix4x4();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                RMatrix[i, j] = R[i, j];
            }
        }

        // Convert rotation matrix to quaternion
        return QuaternionFromMatrix(RMatrix);
    }

    private void SVD(float[,] a, out float[,] u, out float[,] s, out float[,] v)
    {
        // This is a placeholder for the SVD implementation.
        // You can implement SVD manually or use a library that provides SVD functionality.
        // For simplicity, this example assumes you have an SVD implementation available.
        // Replace this with your actual SVD implementation.
        u = new float[3, 3];
        s = new float[3, 3];
        v = new float[3, 3];
    }

    private float[,] MultiplyMatrices(float[,] a, float[,] b)
    {
        int aRows = a.GetLength(0);
        int aCols = a.GetLength(1);
        int bRows = b.GetLength(0);
        int bCols = b.GetLength(1);

        if (aCols != bRows)
            throw new System.Exception("Matrix dimensions are not valid for multiplication.");

        float[,] result = new float[aRows, bCols];

        for (int i = 0; i < aRows; i++)
        {
            for (int j = 0; j < bCols; j++)
            {
                for (int k = 0; k < aCols; k++)
                {
                    result[i, j] += a[i, k] * b[k, j];
                }
            }
        }

        return result;
    }

    private float[,] TransposeMatrix(float[,] a)
    {
        int rows = a.GetLength(0);
        int cols = a.GetLength(1);
        float[,] result = new float[cols, rows];

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                result[j, i] = a[i, j];
            }
        }

        return result;
    }

    private Quaternion QuaternionFromMatrix(Matrix4x4 m)
    {
        // Convert a rotation matrix to a quaternion
        return Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
    }
}