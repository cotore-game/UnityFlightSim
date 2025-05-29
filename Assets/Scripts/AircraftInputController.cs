using UnityEngine;

/// <summary>
/// キーボード入力のみで飛行機を操作し、AerodynamicCalculator で算出した空力を適用するクラス
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class AircraftInputController : MonoBehaviour
{
    // 参照コンポーネント
    public AerodynamicCalculator aeroCalc;
    private Rigidbody rb;

    // エンジン・燃料・電源
    [Header("Propulsion & Power")]
    public Transform engine1Transform;
    public Transform engine2Transform;
    public float maxEngineThrust = 200000f; // N
    public float fuelMass = 20000f;         // kg
    public float fuelConsumptionRate = 0.0001f; // kg per N·s
    public float batteryCharge = 100f;      // %
    public float batteryConsumptionRate = 1f; // % per second when AP on

    // 操作状態
    private float throttleInput = 0f;       // 0〜1
    [SerializeField]
    private bool engine1On = true;
    [SerializeField]
    private bool engine2On = true;
    [SerializeField]
    private bool apuOn = false;
    [SerializeField]
    private bool autoPilotOn = false;

    // オートパイロット目標
    private float apTargetAltitude;
    private Vector3 apTargetVelocity;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (aeroCalc == null) aeroCalc = new AerodynamicCalculator();
        // 初期AP目標を現状に合わせる
        apTargetAltitude = transform.position.y;
        apTargetVelocity = rb.linearVelocity;
    }

    void Update()
    {
        // --- キー入力 ---
        // スロットル (W/S)
        if (Input.GetKey(KeyCode.W))
        {
            throttleInput = Mathf.Min(1f, throttleInput + Time.deltaTime);
            float throttleForce = 800000f; // 推力の強さを調整
            rb.AddRelativeForce(Vector3.forward * throttleForce);
        }

        if (Input.GetKey(KeyCode.S)) throttleInput = Mathf.Max(0f, throttleInput - Time.deltaTime);

        // エンジン1 (Alphaキー: 1)
        if (Input.GetKeyDown(KeyCode.Alpha1)) engine1On = !engine1On;
        // エンジン2 (Alphaキー: 2)
        if (Input.GetKeyDown(KeyCode.Alpha2)) engine2On = !engine2On;
        // APU (Alphaキー: 3)
        if (Input.GetKeyDown(KeyCode.Alpha3)) apuOn = !apuOn;
        // オートパイロット (Alphaキー: 4)
        if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            autoPilotOn = !autoPilotOn;
            if (autoPilotOn)
            {
                apTargetAltitude = transform.position.y;
                apTargetVelocity = rb.linearVelocity;
            }
        }
    }

    void FixedUpdate()
    {
        // --- 燃料消費 ---
        float totalThrust = 0f;
        if (engine1On) totalThrust += maxEngineThrust * throttleInput;
        if (engine2On) totalThrust += maxEngineThrust * throttleInput;
        fuelMass = Mathf.Max(0f, fuelMass - totalThrust * fuelConsumptionRate * Time.fixedDeltaTime);

        // --- バッテリー消費／APU充電 ---
        if (autoPilotOn) batteryCharge = Mathf.Max(0f, batteryCharge - batteryConsumptionRate * Time.fixedDeltaTime);
        if (apuOn && IsOnGround()) batteryCharge = Mathf.Min(100f, batteryCharge + batteryConsumptionRate * Time.fixedDeltaTime);

        // 電源喪失時はAP強制オフ
        if (batteryCharge <= 0f) autoPilotOn = false;

        // --- エンジン推力適用 ---
        Vector3 thrustVector = transform.forward * (maxEngineThrust * throttleInput);
        if (engine1On) rb.AddForceAtPosition(thrustVector, engine1Transform.position);
        if (engine2On) rb.AddForceAtPosition(thrustVector, engine2Transform.position);

        // --- 空力計算・適用 ---
        var result = aeroCalc.CalculateForces(rb);

        if (!IsValidVector(result.force) || !IsValidVector(result.moment))
        {
            Debug.LogWarning($"[AeroCalc] Invalid force or moment detected.\nForce: {result.force}, Moment: {result.moment}");
        }
        else
        {
            rb.AddRelativeForce(result.force);
            rb.AddRelativeTorque(result.moment);
        }

        // --- オートパイロット ---
        if (autoPilotOn)
        {
            // 高度維持
            float altError = apTargetAltitude - transform.position.y;
            rb.AddForce(Vector3.up * altError * 10f);
            // 速度維持
            Vector3 velError = apTargetVelocity - rb.linearVelocity;
            rb.AddForce(velError * 5f);
        }
    }

    private bool IsOnGround()
    {
        return Physics.Raycast(transform.position, Vector3.down, 1.1f);
    }

    private bool IsValidVector(Vector3 v)
    {
        return !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z));
    }
}