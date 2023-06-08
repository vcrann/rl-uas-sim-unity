using UnityEngine;
using UnityEngine.UIElements;
using System.Collections.Generic;

public class MultirotorCreatorManager : MonoBehaviour
{
    private UIDocument _uiDocument;
    private TextField _multirotorName;
    private DoubleField _parameter1;
    private DoubleField _parameter2;
    private DoubleField _parameter3;
    private Button _saveButton;
    private DropdownField _multirotorSelection;
    private void OnEnable()
    {
        _uiDocument = GetComponent<UIDocument>();
        _multirotorName = _uiDocument.rootVisualElement.Q<TextField>("MultirotorNameField");
        _parameter1 = _uiDocument.rootVisualElement.Q<DoubleField>("Parameter1Entry");
        _parameter2 = _uiDocument.rootVisualElement.Q<DoubleField>("Parameter2Entry");
        _parameter3 = _uiDocument.rootVisualElement.Q<DoubleField>("Parameter3Entry");
        _saveButton = _uiDocument.rootVisualElement.Q<Button>("SaveButton");
        _multirotorSelection = _uiDocument.rootVisualElement.Q<DropdownField>("MultirotorSelection");

        //FileManager.LoadFromFile("multirotors.json", out string storedMultirotorsJson);

        //List<MultirotorParameters> storedMultirotors = JsonUtility.FromJson<List<MultirotorParameters>>(storedMultirotorsJson);

        //Debug.Log(storedMultirotors);

        //dropdown.choices = new List<string>();

        _saveButton.clicked += OnClickSave;
    }

    private void OnClickSave()
    {

        MultirotorParameters newMultirotorParameters = new MultirotorParameters(_multirotorName.value, _parameter1.value, _parameter2.value, _parameter3.value);

        //list doesnt seem to work, need wrapper class, look into it further
        List<MultirotorParameters> storedMultirotors = new List<MultirotorParameters>{newMultirotorParameters};
        string json = JsonUtility.ToJson(storedMultirotors);
        Debug.Log(json);

        FileManager.WriteToFile("multirotors.json", json);

        FileManager.LoadFromFile("multirotors.json", out string data);

        Debug.Log(data);
    }
}
