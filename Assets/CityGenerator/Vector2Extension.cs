using UnityEngine;

public static class Vector2Extension {

    public static Vector2 Rotate(this Vector2 v, float degrees)
    {
        float sin = Mathf.Sin(degrees * Mathf.Deg2Rad);
        float cos = Mathf.Cos(degrees * Mathf.Deg2Rad);
        return new Vector2((cos * v.x) - (sin * v.y), (sin * v.x) + (cos * v.y));
    }

    public static Vector2 RotateAroundPoint(this Vector2 v, Vector2 pivot, float degrees)
    {
        return (v-pivot).Rotate(degrees) + pivot;
    }
}
