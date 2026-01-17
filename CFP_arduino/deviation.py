import matplotlib.pyplot as plt
import pandas as pd

# 1. Prepare the data
data = {
    'weight_kg': [0, 0, 0, 1, 1, 1, 2, 2, 2],
    'deviation_pulse': [0, 1, 1, 1, 2, 1, 3, 0, 2]
}

df = pd.DataFrame(data)

# 2. Calculate Mean and Standard Deviation
# We use .agg() to calculate both 'mean' and 'std' at the same time
summary = df.groupby('weight_kg')['deviation_pulse'].agg(['mean', 'std'])

# Fill NaN std with 0 just in case there is only one data point for a weight (not the case here, but good practice)
summary['std'] = summary['std'].fillna(0)

# 3. Create the Plot
plt.figure(figsize=(8, 6))

# Plot the bars with error bars
# yerr=summary['std'] adds the error bars based on the standard deviation
# capsize=5 adds the little horizontal lines at the top/bottom of the error bars
bars = plt.bar(
    summary.index, 
    summary['mean'], 
    yerr=summary['std'], 
    capsize=5, 
    color='skyblue', 
    edgecolor='black',
    alpha=0.8
)

# 4. Formatting
plt.title('Repeatability', fontsize=14)
plt.xlabel('Weight (kg)', fontsize=12)
plt.ylabel('Deviation (pulse)', fontsize=12)
plt.xticks(summary.index) # Ensure x-axis shows integers (0, 1, 2)
plt.grid(axis='y', linestyle='--', alpha=0.5)

# Optional: Print the values on top of bars
for i, row in zip(summary.index, summary.itertuples()):
    # row.mean is the height, row.std is the error size
    # We place the text slightly above the error bar
    plt.text(i, row.mean + row.std + 0.1, f"Avg: {row.mean:.2f}", ha='center', fontsize=9)

plt.show()