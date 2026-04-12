import threading

class EWMAAnomalyModel:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self._lock = threading.Lock()

        # -------------------------
        # PHASE-AWARE BASELINES
        # -------------------------
        self.means = {}          # {phase: {feature: mean}}
        self.prev_values = {}    # {phase: {feature: last_value}}

        # -------------------------
        # PER-SENSOR THRESHOLDS
        # -------------------------
        self.thresholds = {
            "accelerometer": 0.8,
            "flap": 5.0,
            "fueltemp": 1.2,
            "spar": 1.2,
            "slat": 1.5,
            "spoiler": 3.0,
            "flutter": 0.2,
            "wingtip_node": 1.5,
            "wingtip_strain": 0.3,
            "pressure": 0.5
        }

        # -------------------------
        # FEATURE IMPORTANCE (CRITICAL)
        # -------------------------
        self.feature_weights = {
            "damping_ratio": 3.0,
            "snr_dB": 2.5,
            "pressure_gradient_Pa_per_cm": 2.0,
            "tracking_error_deg": 2.0,
            "lag_ms": 2.5,

            # default stable features
            "accel_x": 1.0,
            "accel_y": 1.0,
            "accel_z": 1.0
        }

    # -------------------------
    # UPDATE (ONLINE LEARNING)
    # -------------------------
    def update(self, features, phase, is_anomaly=False):
        if is_anomaly:
            return  # 🔥 prevent drift from faulty data
        
        with self._lock:
            if phase not in self.means:
                self.means[phase] = {}

            for k, v in features.items():
                try:
                    v = float(v)
                except:
                    continue

                if k not in self.means[phase]:
                    self.means[phase][k] = v
                else:
                    self.means[phase][k] = (
                        self.alpha * v + (1 - self.alpha) * self.means[phase][k]
                    )

    # -------------------------
    # PREDICT (ANOMALY SCORE)
    # -------------------------
    def predict(self, features, phase):
        if phase not in self.means:
            return 0

        if phase not in self.prev_values:
            self.prev_values[phase] = {}

        total_error = 0
        count = 0

        for k, v in features.items():
            try:
                v = float(v)
            except:
                continue

            if k in self.means[phase]:
                mean = self.means[phase][k]

                # -------------------------
                # BASE DEVIATION
                # -------------------------
                base_error = abs(v - mean) / (abs(mean) + 1e-6)

                # -------------------------
                # FEATURE WEIGHTING
                # -------------------------
                weight = self.feature_weights.get(k, 1.0)
                error = base_error * weight

                # -------------------------
                # TREND DETECTION (NEW 🔥)
                # -------------------------
                if k in self.prev_values[phase]:
                    prev = self.prev_values[phase][k]
                    trend = abs(v - prev) / (abs(prev) + 1e-6)

                    # amplify gradual drift
                    error += 0.5 * trend

                # store last value
                self.prev_values[phase][k] = v

                total_error += error
                count += 1

        score = total_error / count if count > 0 else 0

        # -------------------------
        # CLAMP EXTREME VALUES
        # -------------------------
        print(f"[PREDICT] Phase={phase}, Score={score}")
        return min(score, 10)

    # -------------------------
    # ANOMALY CHECK
    # -------------------------
    def is_anomaly(self, score, sensor_name):
        threshold = self.thresholds.get(sensor_name, 1.0)
        return score > threshold

    # -------------------------
    # BACKEND UPDATE (SPARK FEEDBACK)
    # -------------------------
    def apply_backend_update(self, sensor_features, phase):
        print("\n🔁 BACKEND UPDATE START")
        print("PHASE:", phase)
        print("FEATURES:", sensor_features)
        print("BEFORE:", self.means.get(phase, {}))
        with self._lock:
            if phase not in self.means:
                self.means[phase] = {}

            for k, v in sensor_features.items():
                try:
                    v = float(v)
                except:
                    continue

                if k in self.means[phase]:
                    self.means[phase][k] = 0.8 * self.means[phase][k] + 0.2 * v
                else:
                    self.means[phase][k] = v 
        print("AFTER:", self.means.get(phase, {}))
        print("🔁 BACKEND UPDATE END\n")
