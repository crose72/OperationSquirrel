import pandas as pd
import plotly.graph_objs as go
import plotly.io as pio
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import os

def select_csv_file():
    """Open a file dialog and select a CSV file."""
    Tk().withdraw()  # Close the root window
    file_path = askopenfilename(filetypes=[("CSV files", "*.csv")])
    return file_path

def load_csv(file_path):
    """Load the data from the selected CSV file."""
    return pd.read_csv(file_path)

def normalize_dataframe(df):
    """Normalize each column of the dataframe to have values between -1 and 1, excluding the 'Time' column."""
    normalized_df = df.copy()
    for column in df.columns:
        if column != 'Time':
            normalized_df[column] = 2 * (df[column] - df[column].min()) / (df[column].max() - df[column].min()) - 1
    return normalized_df

def create_figure(df, max_values, title):
    """Create a Plotly figure with data series from the dataframe."""
    fig = go.Figure()
    for column in df.columns:
        if column != 'Time':
            name_with_max = f"{column} (Max: {max_values[column]:.2f})"
            fig.add_trace(go.Scatter(x=df['Time'], y=df[column], mode='lines', name=name_with_max, line_shape='hv'))
    fig.update_layout(title=title, xaxis_title='Time', yaxis_title='Value')
    return fig

def save_figure(fig, output_file_name):
    """Save the Plotly figure as an HTML file."""
    pio.write_html(fig, file=output_file_name, auto_open=True)

def main():
    csv_file_path = select_csv_file()
    if csv_file_path:
        df = load_csv(csv_file_path)
        
        # Create and save the plain figure
        plain_fig = create_figure(df, df.max(), 'All Data over Time (Plain)')
        plain_output_file_name = os.path.splitext(os.path.basename(csv_file_path))[0] + '_plain.html'
        save_figure(plain_fig, plain_output_file_name)
        
        # Normalize the dataframe and create the normalized figure
        normalized_df = normalize_dataframe(df)
        normalized_fig = create_figure(normalized_df, df.max(), 'All Data over Time (Normalized)')
        normalized_output_file_name = os.path.splitext(os.path.basename(csv_file_path))[0] + '_normalized.html'
        save_figure(normalized_fig, normalized_output_file_name)
    else:
        print("No file selected.")

if __name__ == "__main__":
    main()
