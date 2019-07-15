using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIHandler : MonoBehaviour {

    
	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            CityTest cityScript = FindObjectOfType<CityTest>();
            if (cityScript.myCity != null)
            {
                Destroy(cityScript.myCity.thisCity);
                cityScript.myCity = null;
            }
            cityScript.citySeed = Random.Range(int.MinValue, int.MaxValue);
            gameObject.GetComponentInChildren<InputField>().text = cityScript.citySeed.ToString();
            cityScript.GenerateCity();
        }
	}

    public void NewCity()
    {
        CityTest cityScript = FindObjectOfType<CityTest>();
        if (!cityScript)
        {
            return;
        }


        int seed = 0;
        int.TryParse(gameObject.GetComponentInChildren<InputField>().text, out seed);

        if (cityScript.myCity != null)
        {
            Destroy(cityScript.myCity.thisCity);
            cityScript.myCity = null;
        }

        cityScript.citySeed = seed;

        if (cityScript.citySeed != 0)
                cityScript.GenerateCity();
    }
}
