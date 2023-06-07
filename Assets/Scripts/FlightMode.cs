using UnityEngine;
using UnityEngine.UIElements;

public class FlightMode : MonoBehaviour
{
    private RadioButtonGroup flightModeSelector;
    private Multirotor multirotorScript;

    private void OnEnable()
    {
        var rootVisualElement = GetComponent<UIDocument>().rootVisualElement;

        flightModeSelector = rootVisualElement.Q<RadioButtonGroup>("FlightMode");

        multirotorScript = GameObject.FindGameObjectWithTag("Multirotor").GetComponent<Multirotor>();

        flightModeSelector.RegisterValueChangedCallback(
        (evt) =>
        {
            if (evt.target == flightModeSelector)
                multirotorScript.SetFlightMode(evt.newValue);
        });
    }

}
