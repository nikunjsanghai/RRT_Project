#!/bin/bash

# Create a directory to store the logs
mkdir -p logs

# Get all pod names associated with the rrt-project-job
pods=$(kubectl get pods --selector=job-name=rrt-project-job -o jsonpath='{.items[*].metadata.name}')

# Loop over all the pods and copy their logs
for pod in $pods; do
    echo "Fetching logs for pod: $pod"
    
    # Check if the log-collector container is running, only fetch logs if it is
    if kubectl get pod $pod -o jsonpath='{.status.containerStatuses[?(@.name=="log-collector")].ready}' | grep true; then
        # Try copying logs from the /app/logs/ directory in the log-collector container using tar method
        echo "Running: kubectl exec $pod -c log-collector -- tar cf - /app/logs/ | tar xf - -C logs/$pod/"
        mkdir -p logs/$pod/
        kubectl exec $pod -c log-collector -- tar cf - /app/logs/ | tar xf - -C logs/$pod/
        if [ $? -eq 0 ]; then
            echo "Logs from $pod have been successfully copied to logs/$pod/"
        else
            echo "Failed to copy logs from $pod"
        fi
    else
        echo "log-collector container is not ready in pod: $pod. Skipping log fetch for this pod."
    fi
done

echo "Log fetching completed. Check the logs/ directory for the collected logs."
