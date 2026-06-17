pub trait UniaxialMaterial: Send + Sync {
    fn trial(&mut self, strain: f64) -> (f64, f64);
    fn commit(&mut self);
    fn revert(&mut self);
}

pub struct ElasticSteel {
    pub e: f64,
    pub fy: f64,
    pub strain: f64,
    pub stress: f64,
    pub tangent: f64,
}

impl ElasticSteel {
    pub fn new(e: f64, fy: f64) -> Self {
        Self {
            e,
            fy,
            strain: 0.0,
            stress: 0.0,
            tangent: e,
        }
    }
}

impl UniaxialMaterial for ElasticSteel {
    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let abs_s = strain.abs();
        let (stress, tangent) = if abs_s * self.e <= self.fy {
            (strain * self.e, self.e)
        } else {
            let sgn = if strain >= 0.0 { 1.0 } else { -1.0 };
            (sgn * self.fy, 0.0)
        };
        self.strain = strain;
        self.stress = stress;
        self.tangent = tangent;
        (stress, tangent)
    }

    fn commit(&mut self) {}
    fn revert(&mut self) {}
}

pub struct ElasticConcrete {
    pub e: f64,
    pub fc: f64,
    pub ecu: f64,
    pub strain: f64,
    pub stress: f64,
    pub tangent: f64,
}

impl ElasticConcrete {
    pub fn new(e: f64, fc: f64) -> Self {
        Self {
            e,
            fc,
            ecu: -0.0035,
            strain: 0.0,
            stress: 0.0,
            tangent: e,
        }
    }
}

impl UniaxialMaterial for ElasticConcrete {
    fn trial(&mut self, strain: f64) -> (f64, f64) {
        if strain >= 0.0 {
            self.stress = 0.0;
            self.tangent = 0.0;
        } else if strain >= self.ecu {
            let ratio = strain / self.ecu;
            let c = 2.0 * ratio - ratio * ratio;
            self.stress = c * self.fc;
            self.tangent = (2.0 - 2.0 * ratio) * self.fc / self.ecu.abs();
        } else {
            self.stress = self.fc * (1.0 - 0.15 * (strain - self.ecu) / self.ecu);
            self.tangent = 0.0;
        }
        self.strain = strain;
        (self.stress, self.tangent)
    }

    fn commit(&mut self) {}
    fn revert(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_steel_elastic() {
        let mut s = ElasticSteel::new(205000.0, 235.0);
        let (stress, _) = s.trial(0.001);
        assert!((stress - 205.0).abs() < 1.0);
    }

    #[test]
    fn test_steel_yield() {
        let mut s = ElasticSteel::new(205000.0, 235.0);
        let (stress, _) = s.trial(0.01);
        assert!((stress - 235.0).abs() < 1.0);
    }
}
