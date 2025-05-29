using UnityEngine;

/// <summary>
/// 航空力学モデルを計算するクラス（6DOF対応）
/// </summary>
public class AerodynamicCalculator
{
    // 機体パラメータ
    public float wingArea = 124.0f;            // 翼面積 [m^2]
    public float wingSpan = 34.3f;             // 翼幅 [m]
    public float meanChord = 4.3f;             // 平均翼弦長 [m]
    public float airDensitySeaLevel = 1.225f;  // 海面上空気密度 [kg/m^3]
    public float aspectRatio => wingSpan * wingSpan / wingArea; // アスペクト比

    // 失速迎角 [rad]
    public float stallAngle = 15f * Mathf.Deg2Rad;
    // 揚力係数の線形範囲リフト勾配 [1/rad]
    public float liftSlope = 2.0f * Mathf.PI;
    // 抗力係数の誘導抗力因子 e
    public float oswaldEfficiency = 0.85f;
    // 抗力零次抗力係数
    public float dragZero = 0.02f;

    /// <summary>
    /// 空気密度を高度から計算 [kg/m^3]
    /// </summary>
    public float AirDensity(float altitude)
    {
        // 簡易指数関数モデル
        return airDensitySeaLevel * Mathf.Exp(-altitude / 8500f);
    }

    /// <summary>
    /// 機体座標系における迎角 [rad]
    /// </summary>
    public float CalculateAlpha(Vector3 velocityBody)
    {
        return Mathf.Atan2(velocityBody.y, velocityBody.z);
    }

    /// <summary>
    /// 機体座標系における横滑り角 [rad]
    /// </summary>
    public float CalculateBeta(Vector3 velocityBody)
    {
        return Mathf.Asin(velocityBody.x / velocityBody.magnitude);
    }

    /// <summary>
    /// 揚力、抗力、横力、モーメントを計算し、リターンする
    /// </summary>
    public AerodynamicResult CalculateForces(Rigidbody rb)
    {
        // 速度ベクトルを機体座標系に変換
        Vector3 velBody = rb.transform.InverseTransformDirection(rb.linearVelocity);
        float speed = velBody.magnitude;
        float altitude = rb.transform.position.y;
        float rho = AirDensity(altitude);
        float q = 0.5f * rho * speed * speed;

        // 迎角・横滑り角
        float alpha = CalculateAlpha(velBody);
        float beta = CalculateBeta(velBody);

        // 揚力係数 Cl
        float Cl = (alpha < stallAngle) ? liftSlope * alpha : 0f;
        // 抗力係数 Cd
        float inducedDrag = (Cl * Cl) / (Mathf.PI * aspectRatio * oswaldEfficiency);
        float Cd = dragZero + inducedDrag;
        // 横力係数 Cy
        float Cy = beta * 0.9f;

        // 空力力 (Body系: X前進方向, Y右方向, Z上方向)
        Vector3 aeroForces = new Vector3(
            -q * wingArea * Cd,  // 抗力 (X)
             q * wingArea * Cy,  // 横力 (Y)
             q * wingArea * Cl   // 揚力 (Z)
        );

        // モーメント (簡易モデル、拡張可)
        Vector3 aeroMoments = Vector3.zero;
        // ピッチモーメント M 例: Cm = -Cl * meanChord/4
        aeroMoments.x = -Cl * q * wingArea * meanChord * 0.25f;
        // ロール L, ヨー N (必要に応じて補完)
        aeroMoments.y = beta * q * wingArea * meanChord * 0.1f;
        aeroMoments.z = -beta * q * wingArea * wingSpan * 0.1f;

        return new AerodynamicResult(aeroForces, aeroMoments);
    }
}

/// <summary>
/// 航空力学計算結果を保持する構造体
/// </summary>
public struct AerodynamicResult
{
    public Vector3 force;   // 空力力 (Body系)
    public Vector3 moment;  // 空力モーメント (Body系)

    public AerodynamicResult(Vector3 f, Vector3 m)
    {
        force = f;
        moment = m;
    }
}
