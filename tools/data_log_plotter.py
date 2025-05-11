import pandas as pd
import plotly.graph_objs as go
import plotly.io as pio
from tkinter import Tk
from tkinter.filedialog import askopenfilenames
import os

def select_csv_files():
    """Open a file dialog and select multiple CSV files."""
    Tk().withdraw()  # Close the root window
    file_paths = askopenfilenames(filetypes=[("CSV files", "*.csv")])
    return file_paths

def load_csv(file_path):
    """Load the data from the selected CSV file. Skip empty files."""
    try:
        return pd.read_csv(file_path)
    except pd.errors.EmptyDataError:
        print(f"Skipped empty file: {file_path}")
        return None

def normalize_dataframe(df):
    """Normalize each column of the dataframe to have values between -1 and 1, excluding the 'g_app_elapsed_time' column."""
    normalized_df = df.copy()
    for column in df.columns:
        if column != 'g_app_elapsed_time':
            normalized_df[column] = 2 * (df[column] - df[column].min()) / (df[column].max() - df[column].min()) - 1
    return normalized_df

def create_figure(df, max_values, title):
    """Create a Plotly figure with data series from the dataframe."""
    fig = go.Figure()
    for column in df.columns:
        if column != 'g_app_elapsed_time':
            name_with_max = f"{column} (Max: {max_values[column]:.2f})"
            fig.add_trace(go.Scatter(x=df['g_app_elapsed_time'], y=df[column], mode='lines', name=name_with_max, line_shape='hv'))
    fig.update_layout(title=title, xaxis_title='g_app_elapsed_time', yaxis_title='Value')
    return fig

def save_figure(fig, output_file_name, output_dir):
    """Save the Plotly figure as an HTML file."""
    output_path = os.path.join(output_dir, output_file_name)
    pio.write_html(fig, file=output_path, auto_open=False)

def main():
    csv_file_paths = select_csv_files()
    if csv_file_paths:
        for csv_file_path in csv_file_paths:
            df = load_csv(csv_file_path)
            if df is None:
                continue  # Skip the empty file
            
            output_dir = os.path.dirname(csv_file_path)  # Get the directory of the file
            
            # Create and save the plain figure
            plain_fig = create_figure(df, df.max(), f'All Data over g_app_elapsed_time (Plain) - {os.path.basename(csv_file_path)}')
            plain_output_file_name = os.path.splitext(os.path.basename(csv_file_path))[0] + '_plain.html'
            save_figure(plain_fig, plain_output_file_name, output_dir)
            
            # Normalize the dataframe and create the normalized figure
            normalized_df = normalize_dataframe(df)
            normalized_fig = create_figure(normalized_df, df.max(), f'All Data over g_app_elapsed_time (Normalized) - {os.path.basename(csv_file_path)}')
            normalized_output_file_name = os.path.splitext(os.path.basename(csv_file_path))[0] + '_normalized.html'
            save_figure(normalized_fig, normalized_output_file_name, output_dir)
        
        print(f"Processed {len(csv_file_paths)} files.")
    else:
        print("No files selected.")

if __name__ == "__main__":
    main()
