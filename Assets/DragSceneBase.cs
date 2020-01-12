using System;
using UnityEngine;
using System.Collections;

namespace WTWGAME
{
    public class DragSceneBase : MonoBehaviour
	{
        protected bool m_IsMouseDown = false;
        protected float m_YOnDown = 0;
        protected float _totalMove = 0;
        protected Vector3 _camOriginPos = Vector3.zero;

        protected float _groundPosY = 0;

		public Bounds _bounds=new Bounds(Vector3.zero,Vector3.zero);
		protected float _startDragTime=0;
        //
        bool _bNewDrag = false;
	    const float AUTOSCROLLMAXSPEED = 1000.0f;
        //  
        Vector3 _fingerOrigin = Vector3.zero;
        Vector3 _hitPos = Vector3.zero;

		Vector3 _startPos=Vector3.zero;
        //
        float yaw = 0;
        float pitch = 0;
        float idealYaw = 0;
        float idealPitch = 0;
        public bool clampYawAngle = false;
        public float minYaw = -75;
        public float maxYaw = 75;
        /// </summary>
        public float yawSensitivity = 80.0f;

        /// <summary>
        /// Affects vertical rotation speed
        /// </summary>
        public float pitchSensitivity = 80.0f;
        /// <summary>
        /// Keep pitch angle value between minPitch and maxPitch?
        /// </summary>
        public bool clampPitchAngle = true;
        public float minPitch = -20;
        public float maxPitch = 80;
        public float smoothZoomSpeed = 3.0f;
        public float smoothOrbitSpeed = 4.0f;
        static float ClampAngle(float angle, float min, float max)
        {
            if (angle < -360)
                angle += 360;

            if (angle > 360)
                angle -= 360;

            return Mathf.Clamp(angle, min, max);
        }
        public float IdealYaw
        {
            get { return idealYaw; }
            set { idealYaw = clampYawAngle ? ClampAngle(value, minYaw, maxYaw) : value; }
        }

        public float Pitch
        {
            get { return pitch; }
        }

        public float IdealPitch
        {
            get { return idealPitch; }
            set { idealPitch = clampPitchAngle ? ClampAngle(value, minPitch, maxPitch) : value; }
        }
        public static bool GetHitPos(Vector3 mousePos, out Vector3 hitPos, float groundPosY, bool bUseGemetryHit = true)
        {
            Ray ray = Camera.main.ScreenPointToRay(mousePos);
            //
            // int mask = 1 << 9;
            RaycastHit[] hits = Physics.RaycastAll(ray, 1000);
            hitPos = Vector3.zero;
            if (hits != null && hits.Length > 0)//mask:map
            {
                //Debug.DrawRay(ray.origin, ray.direction * 20, Color.yellow,100);
                hitPos = hits[0].point;
                return true;
            }
            else if (bUseGemetryHit)
            {
                hitPos = GetHitPosWithGeometry(mousePos, Camera.main.transform.position, groundPosY);
            }

            return false;
        }

        //
		IEnumerator MoveToStop(float speed,Vector3 dir,float acc)
        {
			float passedTime = 0;
			dir.y = 0;
			while (!_bNewDrag ) {
				float preTPassedTime = passedTime;
				float dt=Time.deltaTime;
				passedTime = passedTime + dt;

				float nowSpeed = speed + acc * passedTime;
				if (nowSpeed <= 0.0f)
				{
					break;
				}
				else
				{
					float timeParam = preTPassedTime * 2 + dt;
					float offset = (speed + acc * timeParam * 0.5f) * dt;
					Vector3 offsetDir = offset * dir;
					//offsetDir = Camera.main.transform.rotation * offsetDir;
					offsetDir.y = 0;
					
					Vector3 camPos = Camera.main.transform.position +offsetDir;
//					if (!_bounds.Contains (camPos)) {
//						dst=_scale*_bounds.SqrDistance (camPos);
//						camPos= camPos-offsetDir/Math.Max(dst,_fMin);
//						//Debug.Log ("dst1:"+dst);
//						//return;
//					}
					if (!IsInClampPos(ref camPos)) {//!_bounds.Contains (camPos)
				
						break;
					}
                    ChangeCameraTransform(camPos, Quaternion.identity, true,false);
                    //Camera.main.transform.position=camPos;
                }
				yield return null;

			}
			_bNewDrag=true;

            SendCameraPosChangeMsg(false);
         
            //			Tweener tw= transform.DOMove(vec,0.5f);
            //			tw.SetEase (Ease.InExpo);
            //            float fHeight = Mathf.Abs(Camera.main.transform.position.y - _groundPosY);
            //            float fDst = Mathf.Clamp(vec.magnitude, 0, 60 * 15 / fHeight) * 0.1f;
            //            // Debug.Log("dst"+fDst);
            //            //int i = 0;
            //
            //            Vector3 vecoffset = Vector3.zero;
            //            Vector3 rotationVector = Vector3.zero;
            //            float fMin = 0.2f * 15 / fHeight;
            //            while (!_bNewDrag && fDst > fMin)
            //            {
            //                fDst *= (0.9f);//+ i++*0.5f
            //                Vector2 v = vec.normalized * fDst;
            //                vecoffset.Set(v.x, 0, v.y);
            //                Quaternion qua = Quaternion.FromToRotation(this.transform.right, Camera.main.transform.right);
            //                Vector3 temp = qua * vecoffset * fHeight / 34 * 0.55f;// Camera.main.transform.rotation
            //                rotationVector += temp;
            //                if (rotationVector.magnitude >= 0.01f)
            //                {
            //                    Vector3 resultPos = Camera.main.transform.localPosition - rotationVector;
            //
            //                    resultPos.y = m_YOnDown;
            //					if (!_bounds.Contains (resultPos)) {
            //						break;
            //					}
            //                    Camera.main.transform.localPosition = resultPos;
            //                    _totalMove += v.magnitude;
            //                    rotationVector = Vector3.zero;
            //                    SendCameraPosChangeMsg();
            //                }
            //                yield return null;
            //            }
        }
        //
        const float OffsetTime = 0.3f;
        float _preCamTime = 0;
		float _scale=20;
		float _fMin = 2;
        Bounds _tempBound = new Bounds(Vector3.zero, Vector3.zero);
        public virtual void ResetCamPos()
        {
            Vector3 angles =Camera.main.transform.eulerAngles;

            yaw = IdealYaw = angles.y;
            pitch = IdealPitch = angles.x;
            _camPos = Camera.main.transform.position;
            ApplyRotate();
        }
		//Vector3 _start
        public virtual void OnDrag(DragGesture gesture)
        {
            if (Time.time < nextDragTime)
                return;
            _bNewDrag = true;
            //if (m_IsMouseDown == false && UIManager.Instance.IsPointInUI(gesture.Position) == true)
            //    return;

            ContinuousGesturePhase phase = gesture.Phase;

            if (phase == ContinuousGesturePhase.Started)
            {
                m_YOnDown = Camera.main.transform.localPosition.y;
                m_IsMouseDown = true;
				//_camOriginPos = Camera.main.transform.position;
                _fingerOrigin = Camera.main.ScreenToViewportPoint(Input.mousePosition);
				_startPos = Camera.main.transform.position;
				_startDragTime = Time.realtimeSinceStartup;
				_hitPos=GetHitPosWithGeometry(gesture.Position,  Camera.main.transform.position, _groundPosY);

                // ShowCanvas3d(false);
                SendCameraPosChangeMsg(true);
                //Debug.Log("_hitPos0:" + _hitPos);
            }
            else if (phase == ContinuousGesturePhase.Ended)
            {
                //
                _bNewDrag = false;
                if (m_IsMouseDown)
                {
					Vector3 endPos = Camera.main.transform.position;// Camera.main.ScreenToViewportPoint(Input.mousePosition);
					float dst = (endPos - _startPos).magnitude;//???
					float totalDragTime= Time.realtimeSinceStartup-_startDragTime;
					float speed =Math.Min( dst / totalDragTime,AUTOSCROLLMAXSPEED);
					float acc = -200;
					Vector3 dir = (endPos - _startPos);
					StartCoroutine(MoveToStop(speed,dir.normalized,acc));
                }
                else
                {
                    SendCameraPosChangeMsg(false);
         
                }
                m_IsMouseDown = false;
                //
       
            }
            if (phase != ContinuousGesturePhase.Ended && m_IsMouseDown)
            {
				float dst=0;
				//Vector3 pos = GetCamPosByScreenWorldPos (Camera.main, Input.mousePosition, _hitPos);
				//Vector3 offsetCam = -pos + _camOriginPos;
				//Vector3 camPos = _camOriginPos -offsetCam;//
				Vector3 camPos = GetCamPosByScreenWorldPos (Camera.main, Input.mousePosition, _hitPos,_tempBound);
				//if (!_bounds.Contains (camPos)) {
				//	dst = _scale * _bounds.SqrDistance (camPos);
				//	//Vector3 offsetDir = camPos-Camera.main.transform.position;
				//	camPos = Vector3.Lerp (_camOriginPos, camPos, 1.0f / Math.Max (dst, _fMin));
				//	//Debug.Log ("dst1:"+dst);
				//	//return;\
                if(!IsInClampPos(ref camPos))
                {
                    //_camOriginPos = Camera.main.transform.position;
                    //ResetCamPos();                   
                }
                else
                {
                    camPos.y = _startPos.y;
                    ChangeCameraTransform(camPos,Quaternion.identity,true,false);
                }
                //				if (dst / _scale > 0.3f) {
                //					return;
                //				}
         
//                Vector3 mPos = Camera.main.ScreenToViewportPoint(Input.mousePosition);
//                Vector3 dir = mPos - _fingerOrigin;
//                Vector3 camPos = Vector3.zero;
//                dir.z = dir.y;
//                dir.y = 0;

                //
//                float factor = 300 / Mathf.Abs(_hitPos.y - Camera.main.transform.position.y);
//                Vector3 offsetCam = Camera.main.transform.rotation * dir * factor;
//                offsetCam.y = 0;
//                camPos = _camOriginPos - offsetCam;//
//				float dst=0;
//				if (!_bounds.Contains (camPos)) {
//					dst=_scale*_bounds.SqrDistance (camPos);
//					camPos= camPos - offsetCam/Math.Max(dst,_fMin);
//					Debug.Log ("dst..1:"+dst/_scale+",camPos:"+camPos);
//					//return;
//				}
//				if (dst / _scale > 0.3f) {
//					return;
//				}
//				Vector3 cPos = Camera.main.transform.position;
//                Camera.main.transform.position = camPos;//
//                                                        //
//                Vector3 curHit = GetHitPosWithGeometry(gesture.Position, camPos, _hitPos.y);
//
//                Vector3 offset = curHit - _hitPos;
//                offset.y = 0;
//				Vector3 resultPos = camPos - offset;
//				if (!_bounds.Contains (resultPos)) {
//					dst=_scale*_bounds.SqrDistance (resultPos);
//					resultPos= camPos - offset/Math.Max(dst,_fMin);
//					//Debug.Log ("dst:"+dst);
//						//return;
//				}
				//SendCameraPosChangeMsg();
//				if (dst / _scale >  0.3f) {
//					Camera.main.transform.position = cPos;
//					return;
//				}
			//	Camera.main.transform.position = resultPos;

              //  curHit = GetHitPosWithGeometry(gesture.Position, Camera.main.transform.position, _hitPos.y);
             
                //
            }
        }

        public virtual void ChangeCameraTransform(Vector3 pos,Quaternion qua,bool bPos,bool bSetQua)
        {
            if(bPos)
            {
                if(pos.z<34.3f)
                {
                    return;
                }
                Camera.main.transform.position = pos;
            }
            if (bSetQua)
            {
                Camera.main.transform.rotation = qua;
            }
        }
        public bool allowTwoFingerRotate = false;
        protected float nextDragTime;

        protected bool _bRotating = false;
        Vector3 _rotateCenter = Vector3.zero;
        Vector3 _camPos = Vector3.zero;
        Vector3 _IdealCamPos = Vector3.zero;
        protected virtual void OnTwoFingerDrag(PinchRotateGesture gesture)
        {
            //Debug.Log( "OnTwoFingerDrag " + e.Phase + " @ Frame " + Time.frameCount );
            if (allowTwoFingerRotate)// && gesture.DeltaMove.SqrMagnitude()>0
            {
               
                nextDragTime = Time.time + 0.25f;
                if (!_bRotating && gesture.Phase == ContinuousGesturePhase.Started)
                {
                    _bRotating = true;
                    _camPos = Camera.main.transform.position;
                   //StopCoroutine(ApplyRotate());
                   StartCoroutine(ApplyRotate());
                   
                    Vector3 screenPos = Vector3.zero;
                    screenPos.y = Screen.height * 0.5f;
                    screenPos.x = Screen.width * 0.5f;
                    _rotateCenter = GetHitPosWithGeometry(screenPos, Camera.main.transform.position, 0);
                   
                }
                IdealYaw = gesture.DeltaMove.x * yawSensitivity * 0.02f;
                //  += delatAngle;
                float delatAngle = gesture.DeltaMove.y * pitchSensitivity * 0.02f;

                Quaternion preQua = Camera.main.transform.rotation;
                Vector3 prePos = Camera.main.transform.position;
             
       
                //if (IsRotateCenterValid(_rotateCenter) && IdealYaw== fAngle && _IdealCamPos.y>8&& _IdealCamPos.y<30)
                //{
                  Camera.main.transform.RotateAround(_rotateCenter, Vector3.forward, delatAngle);
                //}
                float fAngle = Camera.main.transform.rotation.eulerAngles.x;
                IdealPitch = fAngle;

                _IdealCamPos = Camera.main.transform.position;
                //IsInClampPos(ref _IdealCamPos);
                //  _IdealCamPos.y = prePos.y;

                Camera.main.transform.position = prePos;
               Camera.main.transform.rotation = preQua;
            }
            if (gesture.Phase == ContinuousGesturePhase.Ended)
            {
                _bRotating = false;
            }
        }
        public virtual bool IsRotateCenterValid(Vector3 pos)
        {
            return true;
        }
        IEnumerator ApplyRotate()
        {
            yield return null;
            while (_bRotating)
            {
                yaw = Mathf.LerpAngle(yaw, IdealYaw, Time.deltaTime * smoothOrbitSpeed);
                pitch = Mathf.LerpAngle(pitch, IdealPitch, Time.deltaTime * smoothOrbitSpeed);
                _camPos = Vector3.Lerp(_camPos, _IdealCamPos, Time.deltaTime * smoothOrbitSpeed);
                //_camPos.y=_IdealCamPos.y;
                //. Camera.main.transform.rotation =;
                if(IsInClampPos(ref _camPos))
                {
                    ChangeCameraTransform(_camPos, Quaternion.Euler(pitch, yaw, 0), IsRotateCenterValid(_rotateCenter), true);
                }              
                yield return null;
            }
        }
        //void ShowCanvas3d(bool bShow)
        //{
        //    BattleManager.Instance.Canvas3D.enabled = bShow;
        //}
        float _camDst = 0;
        protected bool IsInClampPos(ref Vector3 camPos)

        {
            //(_bounds.max.x < camPos.x || _bounds.min.x > camPos.x) && (_bounds.max.z < camPos.z || _bounds.min.z > camPos.z)
            //float camDst = _bounds.SqrDistance(camPos);
            //bool bIn = _bounds.Contains(camPos);
            //if (!bIn && camDst >=_camDst)
            //{//!_bounds.Contains (camPos)
            //    float dst = _scale * _bounds.SqrDistance(camPos);
            //    camPos = Vector3.Lerp(_camOriginPos, camPos, 1.0f / Math.Max(dst, _fMin));
            //    //Debug.Log ("dst1:"+dst);
            //    //return;
            //    //break;
            //    _camDst = camDst;
            //    return false;
            //}
            //_camDst = bIn?0:camDst;

            Vector3 screenPos = Vector3.zero;
            screenPos.x = Screen.width * 0.5f;
            screenPos.y = Screen.height*0.5f;
            Vector3 cPos = Camera.main.transform.position;
            Camera.main.transform.position = camPos;
               
            Vector3 vPos = GetHitPosWithGeometry(screenPos, camPos, 0);
            Camera.main.transform.position = cPos;
            bool bIn = _bounds.Contains(vPos);
            if(!bIn)
            {
                if(vPos.x<_bounds.max.x && vPos.x>_bounds.min.x)
                {
                    camPos.z = cPos.z;
                    return true;
                }
                if (vPos.z < _bounds.max.z && vPos.z > _bounds.min.z)
                {
                    camPos.x = cPos.x;
                    return true;
                }
                return false;
            }
            return bIn;
            //  camPos.Set(Mathf.Clamp(camPos.x, _bounds.min.x, _bounds.max.x),Mathf.Clamp(camPos.y, _bounds.min.y, _bounds.max.y), Mathf.Clamp(camPos.z, _bounds.min.z, _bounds.max.z));
            // return true;
        }
        //waitne _wEFrame=new WaitForEndOfFrame();
        IEnumerator DelayBroadCastPosChangsMsg()
        {
            yield return null;     
           // ShowCanvas3d(true);
            yield return null;
            //Messenger.Broadcast(CommonString.MainCameraPosChangeEnd);
            _preCamTime = Time.realtimeSinceStartup;
            Debug.LogWarning("DelayBroadCastPosChangsMsg");
          
        }
        public void SendCameraPosChangeMsg(bool bStart)
        {           
            //if(Time.realtimeSinceStartup- _preCamTime>OffsetTime)
            {
               // StartCoroutine(DelayBroadCastPosChangsMsg());
            }
          //  Messenger.Broadcast(bStart ? CommonString.MainCameraPosChangeStart : CommonString.MainCameraPosChangeEnd);
        }
        public static Vector3 GetHitPosWithGeometry(Vector3 geturePos, Vector3 camPos, float hitY)
        {

            Vector3 pos = geturePos;
            pos.z = Camera.main.nearClipPlane;
			//Vector3 mWPos = ScreenToWorldPoint (Camera.main,pos);

            Vector3 vPos = Camera.main.ScreenToWorldPoint(pos);
            Vector3 dir1 = vPos - camPos;
            Vector3 curHit = Vector3.zero;
            curHit.Set((-camPos.y + hitY) / dir1.y * dir1.x + camPos.x, hitY, (-camPos.y + hitY) / dir1.y * dir1.z + camPos.z);
            return curHit;
        }
        public static Vector3 GetCamPosFromTerrainPos(Vector3 tPos,Vector3 dir, float camPosY)
        {
            Vector3 dirN= dir.normalized;
            float fLen=Mathf.Abs(camPosY/Vector3.Dot(dirN,Vector3.up));
            return tPos + fLen * dirN;
        }
            public static Vector3 GetCamPosByScreenWorldPos(Camera cam,Vector3 screenPos,Vector3 worldHitPos,Bounds b)
		{
			screenPos.z=  cam.nearClipPlane;
			Vector3 center = cam.ScreenToWorldPoint (new Vector3(Screen.width*0.5f,Screen.height*0.5f,cam.nearClipPlane) );
			Vector3 norm=center-cam.transform.position;
			screenPos.z = cam.nearClipPlane;
			Vector3 screenWorldPos = cam.ScreenToWorldPoint (screenPos);
            if(b.size!=Vector3.zero)
            {
                screenWorldPos.Set(Mathf.Clamp(screenWorldPos.x, b.min.x, b.max.x), screenWorldPos.y, Mathf.Clamp(screenWorldPos.z, b.min.z, b.max.z));
            }
         
			//linen formation:p(t)=p0+t* lineDir;
			//plane formation:dot(p,norm)=not(center,norm);
			float d=Vector3.Dot(center,norm);
			Vector3 lineDir = cam.transform.position - screenWorldPos;
			float t=(d-Vector3.Dot(worldHitPos,norm))/Vector3.Dot(lineDir,norm);
			Vector3 pos = worldHitPos + t * lineDir;//near plane pos

			return pos +(cam.transform.position-screenWorldPos);
			//vecRT = cam.transform.rotation * vecRT;
		}
		public static Vector3 ScreenToWorldPoint(Camera cam,Vector3 screenPos)
		{
			Vector3 camPos = Vector3.zero;//cam.transform.position;
			float z = camPos.z + cam.nearClipPlane;

			float fH = cam.nearClipPlane * Mathf.Tan (Mathf.Deg2Rad * cam.fieldOfView * 0.5f);
			float minY = camPos.y - fH;
			float maxY = camPos.y + fH;
			float scale=Screen.width/Screen.height;
			float minX=camPos.x-fH*scale;
			float maxX=camPos.x+fH*scale;

			Vector3 vecLB = new Vector3 (minX,minY,z);
			Vector3 vecRT = new Vector3 (maxX,maxY,z);		

			Vector3 pos = Vector3.zero;
			pos.x = Mathf.Lerp (vecLB.x, vecRT.x, screenPos.x / Screen.width);
			pos.y = Mathf.Lerp (vecLB.y, vecRT.y, screenPos.y / Screen.height);
			pos.z = z;


			pos = cam.transform.transform.TransformPoint( pos);
			return pos;
			//vecRT = cam.transform.rotation * vecRT;
		}
    }
}