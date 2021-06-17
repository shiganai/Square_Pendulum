using UnityEngine;
using System.Collections;

public class drag_Control : MonoBehaviour
{
	public RectTransform panel;
	public RectTransform canvas;

	private float left_Limit;
	private float right_Limit;
	private float bottom_Limit;
	private float top_Limit;

	private float x_Value = 0;
	private float y_Value = 0;

	private void Start()
	{
		left_Limit = panel.anchoredPosition.x;
		right_Limit = left_Limit + canvas.sizeDelta.x * panel.localScale.x;

		bottom_Limit = panel.anchoredPosition.y;
		top_Limit = bottom_Limit + canvas.sizeDelta.y * panel.localScale.y;


		x_Value = 0;
		y_Value = 0;

        transform.position = new Vector3((left_Limit + right_Limit) / 2, (bottom_Limit + top_Limit) / 2, 0);


        var test = new matrix(2, 2);

		//float target_Deg = 10;
		//test[0][0] = Mathf.Cos(target_Deg * Mathf.Deg2Rad);
		//test[1][0] = Mathf.Sin(target_Deg * Mathf.Deg2Rad);
		//test[0][1] = -Mathf.Sin(target_Deg * Mathf.Deg2Rad);
		//test[1][1] = Mathf.Cos(target_Deg * Mathf.Deg2Rad);
		test[0,0] = 1;
		test[1,0] = 1;
		test[0,1] = 0;
		test[1,1] = 1;

		var test_Inverted = matrix.Inverse(test);

		Debug.Log(matrix.asString(test));
		Debug.Log(matrix.asString(test_Inverted));
	}
    public void MyPointerDownUI()
	{
		Debug.Log("‰Ÿ‚³‚ê‚½");
		Debug.Log(transform.position);
	}

	public void MyDragUI()
	{
		Vector3 target_Position = Input.mousePosition;

		if (target_Position.x < left_Limit)
		{
			target_Position.x = left_Limit;
		}
		else if (target_Position.x > right_Limit)
		{
			target_Position.x = right_Limit;
		}
		if (target_Position.y < bottom_Limit)
		{
			target_Position.y = bottom_Limit;
		}
		else if (target_Position.y > top_Limit)
		{
			target_Position.y = top_Limit;

		}

		transform.position = target_Position;

		x_Value = (target_Position.x - left_Limit) / (right_Limit - left_Limit) * 2 - 1;
		y_Value = (target_Position.y - bottom_Limit) / (top_Limit - bottom_Limit) * 2 - 1;
	}

	public float[] get_Graph_Value()
    {
		return new float[] { x_Value, y_Value };
    }
}
