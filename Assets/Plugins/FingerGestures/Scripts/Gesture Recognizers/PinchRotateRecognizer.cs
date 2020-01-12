using UnityEngine;
using System.Collections;
public class PinchRotateGesture : ContinuousGesture
{
    float delta = 0;
    float gap = 0;

    /// <summary>
    /// Gap difference since last frame
    /// </summary>
    public float Delta
    {
        get { return delta; }
        internal set { delta = value; }
    }

    /// <summary>
    /// Current gap distance between the two pinching fingers
    /// </summary>
    public float Gap
    {
        get { return gap; }
        internal set { gap = value; }
    }
    Vector2 deltaMove = Vector2.zero;
    internal Vector2 LastDelta = Vector2.zero;
    internal Vector2 LastPos = Vector2.zero;
    /// <summary>
    /// Distance dragged since last frame
    /// </summary>
    public Vector2 DeltaMove
    {
        get { return deltaMove; }
        internal set { deltaMove = value; }
    }
}
/// <summary>
/// Pinch Gesture Recognizer
///   Two fingers moving closer or further away from each other
/// </summary>
[AddComponentMenu("FingerGestures/Gestures/PinchRotateRecognizer")]
public class PinchRotateRecognizer : ContinuousGestureRecognizer<PinchRotateGesture>
{
    /// <summary>
    /// Pinch DOT product treshold - this controls how tolerant the pinch gesture detector is to the two fingers
    /// moving in opposite directions.
    /// Setting this to -1 means the fingers have to move in exactly opposite directions to each other.
    /// this value should be kept between -1 and 0 excluded.
    /// </summary>    
    public float MinDOT = -0.7f;

    /// <summary>
    /// Minimum pinch distance required to trigger the pinch gesture
    /// </summary>
    public float MinDistance = 5.0f;

    /// <summary>
    /// How much to scale the internal pinch delta by before raising the OnPinchMove event
    /// </summary>
    public float DeltaScale = 1.0f;

    public override string GetDefaultEventMessageName()
    {
        return "OnPinch";
    }

    // Only support 2 simultaneous fingers right now
    public override int RequiredFingerCount
    {
        get { return 2; }
        set { Debug.LogWarning( "Not Supported" ); }
    }

    // TEMP: multi-gesture tracking is not supported for Pinch gesture yet
    public override bool SupportFingerClustering
    {
        get { return false; }
    }

    protected override GameObject GetDefaultSelectionForSendMessage(PinchRotateGesture gesture )
    {
        return gesture.StartSelection;
    }

    public override GestureResetMode GetDefaultResetMode()
    {
        return GestureResetMode.NextFrame;
    }

    protected override bool CanBegin(PinchRotateGesture gesture, FingerGestures.IFingerList touches )
    {
        if( !base.CanBegin( gesture, touches ) )
            return false;

        //FingerGestures.Finger finger0 = touches[0];
        //FingerGestures.Finger finger1 = touches[1];
        //float curGap = Vector2.Distance(finger0.Position, finger1.Position);
        //if (!FingerGestures.AllFingersMoving(finger0, finger1) && curGap > MinDistance)
        //{
        //    Debug.Log("false curGap£º"+ curGap);
        //    return false;
        //}


        //if( !FingersMovedInOppositeDirections( finger0, finger1) && curGap >MinDistance)
        //{
        //    Debug.Log("false1 curGap£º" + curGap);
        //    return false;
        //}

        //if (curGap < MinDistance && touches.AllMoving() && !touches.MovingInSameDirection(0.35f) )
        //{
        //    Debug.Log("false2 curGap£º" + curGap);
        //    return false;
        //}
        //float startGap = Vector2.SqrMagnitude( finger0.StartPosition - finger1.StartPosition );
        //float curGap = Vector2.SqrMagnitude( finger0.Position - finger1.Position );

        //if( FingerGestures.GetAdjustedPixelDistance( Mathf.Abs( startGap - curGap ) ) < ( MinDistance * MinDistance ) )
        //    return false;

        return true;
    }

    protected override void OnBegin(PinchRotateGesture gesture, FingerGestures.IFingerList touches )
    {
        FingerGestures.Finger finger0 = touches[0];
        FingerGestures.Finger finger1 = touches[1];

        gesture.StartPosition = 0.5f * ( finger0.StartPosition + finger1.StartPosition );
        gesture.Position = 0.5f * ( finger0.Position + finger1.Position );

        gesture.Gap = Vector2.Distance( finger0.StartPosition, finger1.StartPosition );
        float curGap = Vector2.Distance( finger0.Position, finger1.Position );
        gesture.Delta = FingerGestures.GetAdjustedPixelDistance( DeltaScale * ( curGap - gesture.Gap ) );
        gesture.Gap = curGap;

        gesture.DeltaMove = gesture.Position - gesture.StartPosition;
        gesture.LastPos = gesture.Position;
        gesture.LastDelta = Vector2.zero;
    }
    readonly string EventOnPinch = "OnPinch";
    readonly string OnTwoFingerDrag = "OnTwoFingerDrag";
    protected override GestureRecognitionState OnRecognize(PinchRotateGesture gesture, FingerGestures.IFingerList touches )
    {
        if( touches.Count != RequiredFingerCount )
        {
            gesture.Delta = 0;

            // fingers were lifted?
            if( touches.Count < RequiredFingerCount )
                return GestureRecognitionState.Recognized;

            // more fingers added, gesture failed
            return GestureRecognitionState.Failed;
        }

        FingerGestures.Finger finger0 = touches[0];
        FingerGestures.Finger finger1 = touches[1];

        gesture.Position = 0.5f * ( finger0.Position + finger1.Position );

        float curGap = Vector2.Distance(finger0.Position, finger1.Position);
       
        // dont do anything if both fingers arent moving
        if ( !FingerGestures.AllFingersMoving( finger0, finger1 ))// && curGap >= UnityEngine.Screen.width * 0.25f
            return GestureRecognitionState.InProgress;

        
        float newDelta = FingerGestures.GetAdjustedPixelDistance( DeltaScale * ( curGap - gesture.Gap ) );
        gesture.Gap = curGap;

       // if( Mathf.Abs( newDelta ) > 0.001f )
        {
            if (touches.AllMoving() && touches.MovingInSameDirection(0.35f))//&& curGap < UnityEngine.Screen.width*0.25f
            {
                EventMessageName = OnTwoFingerDrag;
                //Debug.Log("OnTwoFingerDrag rec");
                // skip without firing event
                // return GestureRecognitionState.InProgress;// GestureRecognitionState.InProgress; //TODO: might want to make this configurable, so the recognizer can fail if fingers move in same direction
            }
            else if(touches.AllMoving() && FingersMovedInOppositeDirections(finger0, finger1))
            {
                EventMessageName = EventOnPinch;
             //   Debug.Log("EventOnPinch rec");
            }
            else
            {
              //  Debug.Log("pinch rotate inProgress");
                return GestureRecognitionState.InProgress;
            }
            gesture.LastDelta = gesture.DeltaMove;
            gesture.Delta = newDelta;
            gesture.DeltaMove = gesture.Position - gesture.LastPos;

            // if we are currently moving, or we were still moving last frame (allows listeners to detect when the finger is stationary when MoveDelta = 0)...
            if (gesture.DeltaMove.sqrMagnitude > 0 || gesture.LastDelta.sqrMagnitude > 0)
                gesture.LastPos = gesture.Position;
            RaiseEvent( gesture );
        }

        return GestureRecognitionState.InProgress;
    }

    #region Utils

    bool FingersMovedInOppositeDirections( FingerGestures.Finger finger0, FingerGestures.Finger finger1 )
    {
        return FingerGestures.FingersMovedInOppositeDirections( finger0, finger1, MinDOT );
    }

    #endregion
}
