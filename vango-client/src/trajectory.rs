use anyhow;
use diff_drive::rigid2d::Vector2D;

pub struct Path {
    waypoints: Vec<Vector2D<f32>>,
}

// TODO: implement more basic paths
impl Path {
    pub fn semi_circle(radius: f32, npoints: usize) -> Self {
        let traj_x = linspace(0.0, radius * 2.0, npoints);
        let mut traj_y: Vec<f32> = vec![];
        for xi in &traj_x {
            let y = (radius.powf(2.0) - (*xi as f32 - radius).powf(2.0)).sqrt();
            traj_y.push(y);
        }

        let mut waypoints: Vec<Vector2D<f32>> = vec![];
        for i in 0..traj_x.len() {
            waypoints.push(Vector2D::new(traj_x[i], traj_y[i]));
        }

        Self { waypoints }
    }

    pub fn write_to_csv(&self, filename: &str) -> anyhow::Result<()> {
        let mut traj_file = csv::Writer::from_path(filename)?;
        for i in 0..self.waypoints.len() {
            let line = [self.waypoints[i].x, self.waypoints[i].y].map(|e| e.to_string());
            traj_file.write_record(line)?;
        }
        Ok(())
    }

    pub fn to_vec(&self) -> Vec<Vector2D<f32>> {
        self.waypoints.clone()
    }
}

fn linspace(start: f32, stop: f32, num_points: usize) -> Vec<f32> {
    let step = (stop - start) / (num_points - 1) as f32;
    (0..num_points).map(|i| start + (i as f32) * step).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_semi_circle_path() {
        let path = Path::semi_circle(0.1, 10);
        let _ = path.write_to_csv("semi-circle.csv");
    }
}

// fn create_ref_traj() -> anyhow::Result<(Vec<f32>, Vec<f32>)> {
//     let mut traj_file = csv::Writer::from_path("traj.csv")?;
//
//     // create a semi-circle trajectory
//     const RADIUS: f32 = 2.0;
//     let traj_x = linspace(0.0, RADIUS * 2.0, 100);
//     let mut traj_y: Vec<f32> = vec![];
//     for xi in &traj_x {
//         let y = (RADIUS.powf(2.0) - (*xi as f32 - RADIUS).powf(2.0)).sqrt();
//         traj_y.push(y);
//     }
//
//     for i in 0..traj_x.len() {
//         let line = [traj_x[i], traj_y[i]].map(|e| e.to_string());
//         traj_file.write_record(line)?;
//     }
//
//     traj_file.flush()?;
//     Ok((traj_x, traj_y))
// }

// fn read_csv_trajectory(csv_path: &str) -> anyhow::Result<Vec<Vector2D<f64>>> {
//     let mut reader = csv::Reader::from_path(csv_path)?;
//
//     let mut points: Vec<Vector2D<f64>> = vec![];
//
//     // note this skips the first line of the csv file
//     for res in reader.records() {
//         let record = res?;
//         let x: f64 = record[0].parse().unwrap();
//         let y: f64 = record[1].parse().unwrap();
//         let v = Vector2D::new(x, y);
//         points.push(v);
//     }
//
//     Ok(points)
// }
