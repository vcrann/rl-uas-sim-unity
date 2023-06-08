[System.Serializable]
public class MultirotorParameters
{   
    public string name;
    public double parameter1;
    public double parameter2;
    public double parameter3;
    public MultirotorParameters(string inputName, double inputParameter1, double inputParameter2, double inputParameter3)
    {
        name = inputName;
        parameter1 = inputParameter1;
        parameter2 = inputParameter2;
        parameter3 = inputParameter3;
    }
}