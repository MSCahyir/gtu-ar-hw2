using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;
using Button = UnityEngine.UI.Button;

public class UIController : MonoBehaviour
{
    public List<PointCloudReader> pointCloudReader;
    public RigidTransformation rigidTransformation;
    public ScaleTransformation scaleTransformation;
    public Button loadSet1Button;
    public Button loadSet2Button;
    public Button RigidButton;
    public Button ScaleButton;
    public Button resetButton;

    void Start()
    {
        if (pointCloudReader.Count < 3)
        {
            Debug.LogError("Not enough PointCloudReader components assigned to UIController.");
            return;
        }
        // Add listener to the button
        loadSet1Button.onClick.AddListener(LoadSet1);
        loadSet2Button.onClick.AddListener(LoadSet2);
        RigidButton.onClick.AddListener(RigidCall);
        ScaleButton.onClick.AddListener(ScaleCall);
        resetButton.onClick.AddListener(ResetVisualizations);
    }

    private void LoadSet1()
    {
        // Ensure the PointCloudReader has a filePath set
        if (string.IsNullOrEmpty(pointCloudReader[0].filePath) && string.IsNullOrEmpty(pointCloudReader[1].filePath))
        {
            Debug.LogError("File path is not set in PointCloudReader.");
            return;
        }

        // Initialize and visualize points
        pointCloudReader[0].Initialize();
        pointCloudReader[1].Initialize();
        pointCloudReader[0].VisualizePoints();
        pointCloudReader[1].VisualizePoints();

        rigidTransformation.SetPoints(pointCloudReader[0].pointsSet, pointCloudReader[1].pointsSet);
        scaleTransformation.SetPoints(pointCloudReader[0].pointsSet, pointCloudReader[1].pointsSet);
    }

    private void LoadSet2()
    {
        // Ensure the PointCloudReader has a filePath set
        if (string.IsNullOrEmpty(pointCloudReader[0].filePath) && string.IsNullOrEmpty(pointCloudReader[2].filePath))
        {
            Debug.LogError("File path is not set in PointCloudReader.");
            return;
        }

        // Initialize and visualize points
        pointCloudReader[0].Initialize();
        pointCloudReader[2].Initialize();
        pointCloudReader[0].VisualizePoints();
        pointCloudReader[2].VisualizePoints();

        rigidTransformation.SetPoints(pointCloudReader[0].pointsSet, pointCloudReader[2].pointsSet);
        scaleTransformation.SetPoints(pointCloudReader[0].pointsSet, pointCloudReader[2].pointsSet);
    }

    private void RigidCall()
    {
        rigidTransformation.Driver();
    }

    private void ScaleCall()
    {
        scaleTransformation.Driver();
    }

    private void ResetVisualizations()
    {
        // Destroy all instances of sphere objects
        foreach (var sphere in GameObject.FindGameObjectsWithTag("reset"))
        {
            Destroy(sphere);
        }
    }

}
