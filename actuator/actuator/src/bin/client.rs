use actuator::client::run_dummy_client;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    run_dummy_client().await?;
    Ok(())
}
