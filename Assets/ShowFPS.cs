using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using TrueSync;
public class ShowFPS : MonoBehaviour
{

    public float fpsMeasuringDelta = 2.0f;

    private float timePassed;
    private int m_FrameCount = 0;
    private float m_FPS = 0.0f;

    public Text _fps;
    private void Start()
    {
        timePassed = 0.0f;
        Shadow s = _fps.GetComponent<Shadow>();
        s.effectColor = Color.red;
        Outline o = _fps.GetComponent<Outline>();
        o.effectColor = Color.green;
      
    }

    private void Update()
    {
        m_FrameCount = m_FrameCount + 1;
        timePassed = timePassed + Time.deltaTime;

        if (timePassed > fpsMeasuringDelta)
        {
            float fps = m_FrameCount / timePassed;

            timePassed = 0.0f;
            m_FrameCount = 0;
            if(m_FPS!=fps)
            {
                _fps.text = "FPS:" + fps;
            }
            m_FPS = fps;
        }
    }

}