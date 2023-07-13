use anyhow;

fn linspace(start: f32, stop: f32, num_points: usize) -> Vec<f32> {
    let step = (stop - start) / (num_points - 1) as f32;
    (0..num_points).map(|i| start + (i as f32) * step).collect()
}

fn main() -> anyhow::Result<()> {
    const RADIUS: f32 = 2.0;
    let traj_x = linspace(-RADIUS, RADIUS, 100);
    let mut traj_y: Vec<f32> = vec![];
    for xi in &traj_x {
        let y = (RADIUS.powf(2.0) - (*xi as f32).powf(2.0)).sqrt();
        traj_y.push(y);
    }

    let mut writer = csv::Writer::from_path("data.csv")?;
    for i in 0..traj_x.len() {
        let line = [traj_x[i], traj_y[i]].map(|e| e.to_string());
        writer.write_record(line)?;
    }

    writer.flush()?;

    Ok(())
}
