using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class playersettings : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

        PlayerSettings.SetAdditionalIl2CppArgs("--compiler-flags=\"-fbracket-depth=1048576\"");


    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
