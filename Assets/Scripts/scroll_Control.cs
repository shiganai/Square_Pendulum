using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class scroll_Control : MonoBehaviour
{
    private Scrollbar scrollbar;
    // Start is called before the first frame update
    void Start()
    {
        scrollbar = gameObject.GetComponent<Scrollbar>();
    }

    // Update is called once per frame
    void Update()
    {

    }

    public void OnValueCahged()
    {
        Debug.Log(scrollbar.value);
    }

    public float GetValue()
    {
        return scrollbar.value;
    }
}
