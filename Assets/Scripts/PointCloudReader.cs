using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class PointCloudReader : MonoBehaviour
{
    // Variables to hold file paths
    public string filePath;
    // Prefab and materials for visualization
    public GameObject pointPrefab;
    public Material materialSet;

    public List<Vector3> pointsSet = new List<Vector3>();

    public void Initialize()
    {
        pointsSet = ReadPointsFromFile(filePath);
    }

    public void VisualizePoints()
    {
        VisualizePoints(pointsSet, materialSet);
    }

    private List<Vector3> ReadPointsFromFile(string filePath)
    {
        List<Vector3> points = new List<Vector3>();

        // Check if the file exists
        if (!File.Exists(filePath))
        {
            Debug.LogError("File not found: " + filePath);
            return points;
        }

        string[] lines = File.ReadAllLines(filePath);

        // Read the number of points from the first line
        if (!int.TryParse(lines[0], out int numPoints))
        {
            Debug.LogError("First line is not a number: " + lines[0]);
            return points;
        }

        // Read each point
        for (int i = 1; i <= numPoints; i++)
        {
            if (i >= lines.Length)
            {
                Debug.LogError("Not enough lines in file for expected number of points");
                break;
            }

            string[] coordinates = lines[i].Split(' ');
            if (coordinates.Length == 3)
            {
                try
                {
                    float x = float.Parse(coordinates[0]);
                    float y = float.Parse(coordinates[1]);
                    float z = float.Parse(coordinates[2]);
                    points.Add(new Vector3(x, y, z));
                }
                catch (FormatException e)
                {
                    Debug.LogError("Error parsing line: " + lines[i] + "; " + e.Message);
                }
            }
            else
            {
                Debug.LogError("Incorrect number of coordinates in line: " + lines[i]);
            }
        }

        Debug.Log("Read " + points.Count + " points from " + filePath);
        return points;
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
}
