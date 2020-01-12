using UnityEngine;
using System.Collections;
using UnityEngine.UI;

namespace War.UI
{
    [RequireComponent(typeof(Text))]
    public class FlowText : MonoBehaviour
    {
        const float c_moveSpeed =30;
        Text _text = null;
        enum EmoveStatus
        {
            left,
            //stop,
            right
        }
        // Text _text;
        RectTransform _parent;//parent need have mask component
        RectTransform _tr;
        EmoveStatus _status = EmoveStatus.left;
        float _delaytTime = 0;
        Coroutine _flowAni = null;
        void Awake()
        {
            _parent = transform.parent as RectTransform;
            _tr = transform as RectTransform;
            _text = GetComponent<Text>();
        }
        public string text
        {
            get { return _text.text; }
            set
            {
                _text.text = value;
                UpdateStatus();
                if (_flowAni == null)
                {
                    _flowAni = StartCoroutine(Flow());
                }
            }
        }
        void OnEnable()
        {
            if (_flowAni == null)
            {
                _flowAni = StartCoroutine(Flow());
            }
        }
        void UpdateStatus()
        {
            if(_parent.sizeDelta.x < _tr.sizeDelta.x)
            {
                _status = EmoveStatus.left;
                Vector3 pos = _tr.anchoredPosition3D;
                pos.x = _tr.sizeDelta.x * (_tr.pivot.x) - _parent.sizeDelta.x * _tr.anchorMax.x;
                _tr.anchoredPosition3D = pos;
            }
        }
        WaitForSeconds _wait=  new WaitForSeconds(0.8f);
        IEnumerator Flow()
        {
            yield return null;
            UpdateStatus();
            while (enabled && _parent.sizeDelta.x < _tr.sizeDelta.x)
            {
                if(_delaytTime>0)
                {
                    _delaytTime -= Time.deltaTime;
                }
                else
                {
                    _delaytTime = 0;

                    Vector3 pos = _tr.anchoredPosition3D;
                   
                    if(_status==EmoveStatus.left)
                    {
                        float curRightPos = _tr.sizeDelta.x * (1 - _tr.pivot.x) + _parent.sizeDelta.x * _tr.anchorMax.x
                                       + pos.x;
                        if (curRightPos<=_parent.sizeDelta.x)
                        {
                            yield return _wait;
                            _status = EmoveStatus.right;
                        }
                        else
                        {
                            pos.x -= Time.deltaTime*c_moveSpeed;
                            _tr.anchoredPosition3D = pos;
                        }
                    }
                    else if (_status == EmoveStatus.right)
                    {
                        float curLeftPos = -_tr.sizeDelta.x * (_tr.pivot.x) + _parent.sizeDelta.x * _tr.anchorMax.x
                                       + pos.x;
                        if (curLeftPos >= 0)
                        {
                            yield return _wait;
                            _status = EmoveStatus.left;
                        }
                        else
                        {
                            pos.x += Time.deltaTime * c_moveSpeed;
                            _tr.anchoredPosition3D = pos;
                        }
                    }
                }
                yield return null;
            }
            _flowAni = null;
        }
        void OnDisable()
        {
            _flowAni = null;
        }
    }
}
