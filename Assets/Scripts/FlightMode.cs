using UnityEngine;
using UnityEngine.UIElements;
using UnityEngine.SceneManagement;

public class FlightMode : MonoBehaviour
{
    private Button _createMultirotorButton;
    private RadioButtonGroup _flightModeSelector;
    private Multirotor _multirotorScript;

    private void OnEnable()
    {
        var rootVisualElement = GetComponent<UIDocument>().rootVisualElement;

        _createMultirotorButton = rootVisualElement.Q<Button>("CreateMultirotor");
        _createMultirotorButton.clicked += OnClickCreateMultirotor;

        _flightModeSelector = rootVisualElement.Q<RadioButtonGroup>("FlightMode");

        _multirotorScript = GameObject.FindGameObjectWithTag("Multirotor").GetComponent<Multirotor>();

        _flightModeSelector.RegisterValueChangedCallback(
        (evt) =>
        {
            if (evt.target == _flightModeSelector)
                _multirotorScript.SetFlightMode(evt.newValue);
        });
    }

    private void OnClickCreateMultirotor()
    {
        SceneManager.LoadScene("MultirotorCreator");
    }

}
